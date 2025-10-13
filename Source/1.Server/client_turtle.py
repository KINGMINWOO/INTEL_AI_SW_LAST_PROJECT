"""TurtleBot 전용 스트리밍 클라이언트 (ROS2 제어 제거 버전)."""

import socket
import struct
import sys
import threading
import time
from typing import Optional, Tuple

import cv2

AUTH_PROMPT = "AUTH_REQUEST"
AUTH_OK = "AUTH_OK"
CLIENT_ID = "TURTLE01"
CLIENT_PASSWORD = "TURTLE1234"

SERVER_ADDR: Tuple[str, int] = ("10.10.16.36", 9999)
FRAME_SIZE = (1280, 720)  # (width, height)
TARGET_FPS = 30          # 0 => no sleep, otherwise throttle
JPEG_QUALITY = 85


def configure_capture() -> Tuple[cv2.VideoCapture, float]:
    """카메라를 MJPG 저지연 모드로 설정하고 센서 FPS를 반환."""
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다.")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])

    requested_fps = float(TARGET_FPS) if TARGET_FPS > 0 else 0.0
    if requested_fps > 0:
        cap.set(cv2.CAP_PROP_FPS, requested_fps)
    else:
        cap.set(cv2.CAP_PROP_FPS, 30.0)

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    sensor_fps = cap.get(cv2.CAP_PROP_FPS)

    if requested_fps > 0 and sensor_fps > requested_fps + 1e-3:
        print(
            f"카메라 설정: {actual_w:.0f}x{actual_h:.0f} @ {sensor_fps:.1f}fps"
            f" (요청 {requested_fps:.1f}fps, 전송은 {requested_fps:.1f}fps로 제한)")
    else:
        print(f"카메라 설정: {actual_w:.0f}x{actual_h:.0f} @ {sensor_fps:.1f}fps")

    return cap, sensor_fps


def _recv_line(sock: socket.socket) -> Optional[str]:
    data = bytearray()
    while b"\n" not in data:
        chunk = sock.recv(1024)
        if not chunk:
            return None
        data.extend(chunk)
    try:
        return data.split(b"\n", 1)[0].decode("utf-8").strip()
    except UnicodeDecodeError:
        return None


def authenticate(sock: socket.socket) -> bool:
    prompt = _recv_line(sock)
    if prompt != AUTH_PROMPT:
        print("서버 인증 프롬프트를 받지 못했습니다.")
        return False

    credential = f"{CLIENT_ID}:{CLIENT_PASSWORD}\n".encode("utf-8")
    sock.sendall(credential)

    response = _recv_line(sock)
    if response == AUTH_OK:
        print("서버 인증을 통과했습니다.")
        return True

    print("서버 인증에 실패했습니다. ID/Password를 확인하세요.")
    return False


def handle_server_message(line: str) -> None:
    if not line:
        return
    if line.startswith("MOVE:"):
        print(f"MOVE 명령 수신(무시): {line}")
    elif line.strip().upper() == "STOP":
        print("STOP 명령 수신(참고용).")
    else:
        print(f"서버 메시지: {line}")


def receive_async(sock: socket.socket) -> None:
    buffer = bytearray()
    while True:
        try:
            chunk = sock.recv(1024)
            if not chunk:
                print("서버와의 연결이 끊어졌습니다.")
                break
            buffer.extend(chunk)
            while b"\n" in buffer:
                raw, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    handle_server_message(line)
        except OSError:
            print("메시지 수신 스레드 오류.")
            break


def send_control_message(sock: socket.socket, message: str) -> None:
    text = message.strip()
    if not text:
        return
    payload = (text + "\n").encode("utf-8")
    header = struct.pack(">L", 0)
    try:
        sock.sendall(header + payload)
        print(f"서버로 제어 메시지 전송: {text}")
    except OSError as exc:
        print(f"제어 메시지 전송 실패: {exc}")


def command_prompt(stop_event: threading.Event, sock: socket.socket) -> None:
    print("명령 입력 스레드 시작. 'robot@done' 또는 기타 문자열을 입력하면 서버로 전달됩니다.")
    while not stop_event.is_set():
        try:
            user_input = input().strip()
        except EOFError:
            break
        if not user_input:
            continue
        send_control_message(sock, user_input)


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

    sock.connect(SERVER_ADDR)
    print(f"서버 {SERVER_ADDR[0]}:{SERVER_ADDR[1]}에 연결되었습니다.")

    if not authenticate(sock):
        sock.close()
        sys.exit(1)

    stop_event = threading.Event()

    threading.Thread(target=receive_async, args=(sock,), daemon=True).start()
    threading.Thread(target=command_prompt, args=(stop_event, sock), daemon=True).start()

    try:
        cap, sensor_fps = configure_capture()
    except RuntimeError as exc:
        print(exc)
        sock.close()
        sys.exit(1)

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

    if TARGET_FPS > 0 and sensor_fps > 0:
        stream_fps = min(float(TARGET_FPS), sensor_fps)
    elif TARGET_FPS > 0:
        stream_fps = float(TARGET_FPS)
    else:
        stream_fps = sensor_fps if sensor_fps > 0 else 0.0

    frame_interval = 1.0 / stream_fps if stream_fps > 0 else 0.0

    if TARGET_FPS > 0 and sensor_fps > TARGET_FPS + 1e-3:
        print(f"전송 주기는 {TARGET_FPS}fps 기준으로 제어합니다.")
    elif stream_fps > 0 and TARGET_FPS > 0 and sensor_fps > 0 and sensor_fps < TARGET_FPS - 1e-3:
        print(f"카메라가 {sensor_fps:.1f}fps까지만 지원하여 전송 FPS도 동일하게 제한됩니다.")

    try:
        next_frame_time = time.perf_counter()
        while True:
            cap.grab()
            ret, frame = cap.read()
            if not ret:
                print("프레임을 읽지 못했습니다. 종료합니다.")
                break

            if frame.shape[1] != FRAME_SIZE[0] or frame.shape[0] != FRAME_SIZE[1]:
                frame = cv2.resize(frame, FRAME_SIZE, interpolation=cv2.INTER_LINEAR)

            ok, encoded = cv2.imencode(".jpg", frame, encode_param)
            if not ok:
                print("JPEG 인코딩 실패. 프레임을 건너뜁니다.")
                continue

            payload = encoded.tobytes()
            header = struct.pack(">L", len(payload))
            try:
                sock.sendall(header + payload)
            except OSError:
                print("소켓 전송 중 오류 발생. 종료합니다.")
                break

            if frame_interval > 0:
                next_frame_time += frame_interval
                sleep_for = next_frame_time - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_frame_time = time.perf_counter()
    except KeyboardInterrupt:
        print("사용자 중단 요청으로 종료합니다.")
    finally:
        stop_event.set()
        cap.release()
        sock.close()


if __name__ == "__main__":
    main()

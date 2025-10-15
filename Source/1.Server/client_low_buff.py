# client_low_buff.py
import socket
import struct
import sys
import threading
import time
from typing import Tuple

import cv2

AUTH_PROMPT = "AUTH_REQUEST"
AUTH_OK = "AUTH_OK"
CLIENT_ID = "CCTV1"
CLIENT_PASSWORD = "CCTV1234"

SERVER_ADDR: Tuple[str, int] = ("10.10.16.29", 9999)
FRAME_SIZE = (1280, 720)  # (width, height)
TARGET_FPS = 20           # 0 => no sleep, otherwise throttle
JPEG_QUALITY = 85


def _recv_line(sock: socket.socket) -> str | None:
    """Read a line terminated by newline from the TCP connection."""
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
    """Perform the server-side credential handshake."""
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


def receive_async(sock: socket.socket) -> None:
    """Background listener for server event messages."""
    while True:
        try:
            message = sock.recv(1024)
            if not message:
                print("서버와의 연결이 끊어졌습니다.")
                break
            print(f"서버로부터 받은 메시지: {message.decode('utf-8', errors='ignore')}")
        except OSError:
            print("메시지 수신 스레드 오류.")
            break


def configure_capture() -> cv2.VideoCapture:
    """Configure the camera for low-latency MJPG capture."""
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다.")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])
    cap.set(cv2.CAP_PROP_FPS, max(TARGET_FPS, 30))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"카메라 설정: {actual_w:.0f}x{actual_h:.0f} @ {actual_fps:.1f}fps")
    return cap


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

    sock.connect(SERVER_ADDR)
    print(f"서버 {SERVER_ADDR[0]}:{SERVER_ADDR[1]}에 연결되었습니다.")

    if not authenticate(sock):
        sock.close()
        sys.exit(1)

    threading.Thread(target=receive_async, args=(sock,), daemon=True).start()

    try:
        cap = configure_capture()
    except RuntimeError as exc:
        print(exc)
        sock.close()
        sys.exit(1)

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    frame_interval = 1.0 / TARGET_FPS if TARGET_FPS > 0 else 0.0

    try:
        while True:
            loop_start = time.time()
            cap.grab()  # discard buffered frame to reduce latency
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
                elapsed = time.time() - loop_start
                remaining = frame_interval - elapsed
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        print("사용자 중단 요청으로 종료합니다.")
    finally:
        cap.release()
        sock.close()


if __name__ == "__main__":
    main()

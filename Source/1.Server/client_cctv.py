import socket
import cv2
import struct
import time
import threading
import sys
from collections import deque

# === UART ===
import serial
from serial import Serial
from typing import Optional, Deque

AUTH_PROMPT = "AUTH_REQUEST"
AUTH_OK = "AUTH_OK"
CLIENT_ID = "CCTV01"
CLIENT_PASSWORD = "CCTV1234"

# --- TCP server config ---
server_ip = '192.168.0.4'
server_port = 9999

# --- UART3 config (RPi4B) ---
UART_BAUD = 115200
UART_TIMEOUT = 0.1
UART_PORT_CANDIDATES = [
    "/dev/ttyAMA3",  # common for uart3 overlay
    "/dev/ttyAMA2",  # sometimes numbering differs by DT
    "/dev/ttyS3",    # mini-uart numbering on some kernels
]

stop_event = threading.Event()
uart: Optional[Serial] = None

def _recv_line(sock: socket.socket) -> Optional[str]:
    """Receive a single newline-terminated message from the server."""
    buffer = b""
    while b"\n" not in buffer:
        chunk = sock.recv(1024)
        if not chunk:
            return None
        buffer += chunk
    line, _ = buffer.split(b"\n", 1)
    try:
        return line.decode("utf-8").strip()
    except UnicodeDecodeError:
        return None

def authenticate(sock: socket.socket) -> bool:
    """Perform handshake before starting the streaming loop."""
    prompt = _recv_line(sock)
    if prompt != AUTH_PROMPT:
        print("서버로부터 인증 요청을 받지 못했습니다. 연결을 종료합니다.")
        return False

    credential_payload = f"{CLIENT_ID}:{CLIENT_PASSWORD}\n".encode("utf-8")
    sock.sendall(credential_payload)

    response = _recv_line(sock)
    if response == AUTH_OK:
        print("서버 인증을 통과했습니다.")
        return True

    print("서버 인증에 실패했습니다. ID/Password를 확인하세요.")
    return False

def open_uart3() -> Optional[Serial]:
    """Try to open UART3 on one of the known device nodes."""
    for dev in UART_PORT_CANDIDATES:
        try:
            ser = serial.Serial(
                port=dev,
                baudrate=UART_BAUD,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=UART_TIMEOUT,
            )
            print(f"[UART3] Opened {dev} @ {UART_BAUD}")
            return ser
        except Exception:
            pass
    print("[UART3] Failed to open any UART3 device. Checked:", ", ".join(UART_PORT_CANDIDATES))
    print("        - Ensure `dtoverlay=uart3` in /boot/firmware/config.txt (then reboot).")
    print("        - Check with: ls -l /dev/ttyAMA* /dev/ttyS*")
    return None   

def uart_send_line(text: str):
    """Send one line (LF-terminated) to STM32 via UART3."""
    global uart
    if uart and uart.is_open:
        data = (text.rstrip("\r\n") + "\n").encode("utf-8", errors="ignore")
        try:
            uart.write(data)
        except Exception as e:
            print(f"[UART3] write error: {e}")
    else:
        print("[UART3] not open; cannot send")

def send_line_to_server(sock: socket.socket, text: str):
    """프레임 스트림과 동일한 채널에서 제어 메시지를 보내기 위해 0 길이 헤더를 붙인다."""
    payload = (text.rstrip("\r\n") + "\n").encode("utf-8", errors="ignore")
    header = struct.pack(">L", 0)
    try:
        sock.sendall(header + payload)
    except Exception as e:
        print(f"[NET] send_line_to_server error: {e}")

def uart_reader(sock: socket.socket):
    """Read lines from STM32 and print, then forward to server."""
    global uart
    if not (uart and uart.is_open):
        return
    buf = bytearray()
    while not stop_event.is_set():
        try:
            chunk = uart.read(128)
            if chunk:
                buf.extend(chunk)
                while b"\n" in buf:
                    line, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)
                    try:
                        txt = line.decode("utf-8", errors="ignore").rstrip("\r")
                    except Exception:
                        txt = str(line)
                    if txt:
                        print(f"[UART3 RX] {txt}")
                        send_line_to_server(sock, txt)   # ★ 서버로 그대로 전달
        except Exception as e:
            print(f"[UART3] read error: {e}")
            time.sleep(0.1)

def receive_message(sock: socket.socket):
    buf = bytearray()
    while not stop_event.is_set():
        chunk = sock.recv(1024)
        if not chunk:
            print("서버와의 연결이 끊어졌습니다."); break
        buf.extend(chunk)
        while b"\n" in buf:
            line, _, rest = buf.partition(b"\n")
            buf = bytearray(rest)
            text = line.decode("utf-8", errors="ignore").strip()

            if text.startswith("[CCTV1]"):
                payload = text[7:].lstrip()
                print(f"[CTRL] Forward to UART3: {payload}")
                uart_send_line(payload)
            else:
                print(f"서버로부터 받은 메시지: {text}")


def stdin_relay(sock: socket.socket, sensor_queue: Deque[str], sensor_lock: threading.Lock):
    """사용자가 터미널에서 입력한 센서 페이로드를 큐에 쌓거나 즉시 전송."""
    print("센서 문자열 전송: 'air@...' 또는 'land@...' 형식으로 입력하세요. 종료는 Ctrl+C.")
    while not stop_event.is_set():
        try:
            line = input()
        except (KeyboardInterrupt, EOFError):
            print("표준입력 종료 요청을 받았습니다.")
            stop_event.set()
            break

        line = line.strip()
        if not line:
            continue

        if line.lower().startswith(("air@", "land@")):
            with sensor_lock:
                sensor_queue.append(line)
            print(f"[INPUT] 센서 큐에 추가됨: {line}")
        else:
            send_line_to_server(sock, line)
            print(f"[INPUT] 즉시 전송: {line}")

def main():
    global uart

    # Connect TCP
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_ip, server_port))
    print(f"서버 {server_ip}:{server_port}에 연결되었습니다.")

    if not authenticate(sock):
        sock.close()
        sys.exit(1)

    # Start server receiver
    receiver_thread = threading.Thread(target=receive_message, args=(sock,), daemon=True)
    receiver_thread.start()

    # Open UART3 and start reader
    uart = open_uart3()
    if uart and uart.is_open:
        th = threading.Thread(target=uart_reader, args=(sock,), daemon=True)
        th.start()

    # Open camera (USB webcam at /dev/video0). For Pi Camera, consider picamera2.
#cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        try:
            sock.close()
        except Exception:
            pass
        if uart and uart.is_open:
            try:
                uart.close()
            except Exception:
                pass
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    target_fps = 10
    frame_interval = 1.0 / target_fps

    try:
        last_sensor_tx = 0.0
        SENSOR_INTERVAL = 5.0
        sensor_queue: Deque[str] = deque()
        sensor_lock = threading.Lock()
        threading.Thread(
            target=stdin_relay,
            args=(sock, sensor_queue, sensor_lock),
            daemon=True,
        ).start()

        while not stop_event.is_set():
            loop_start = time.time()
            ret, frame = cap.read()
            if not ret:
                print("프레임을 읽을 수 없습니다. 종료합니다.")
                break

            ok, enc = cv2.imencode(".jpg", frame, encode_param)
            if not ok:
                continue
            payload = enc.tobytes()
            msg_size = struct.pack(">L", len(payload))

            try:
                sock.sendall(msg_size + payload)
            except socket.error:
                print("소켓 오류: 연결이 끊겼습니다.")
                break

            dt = time.time() - loop_start
            if dt < frame_interval:
                time.sleep(frame_interval - dt)
            now = time.time()
            if now - last_sensor_tx >= SENSOR_INTERVAL:
                sample = None
                with sensor_lock:
                    if sensor_queue:
                        sample = sensor_queue.popleft()
                if sample:
                    send_line_to_server(sock, sample)
                    print(f"[센서] 페이로드 전송: {sample}")
                    last_sensor_tx = now

    except (ConnectionAbortedError, ConnectionResetError, socket.error) as e:
        print(f"오류 발생: {e}")
    finally:
        stop_event.set()
        try:
            cap.release()
        except Exception:
            pass
        try:
            sock.close()
        except Exception:
            pass
        if uart and uart.is_open:
            try:
                uart.close()
            except Exception:
                pass
        print("정상 종료")

if __name__ == "__main__":
    main()

# server_highres_output.py (컴퓨터에서 실행)
import socket
import cv2
# import pickle # pickle은 더 이상 사용하지 않음
import struct
import threading
import time
import hmac
from pathlib import Path
import numpy as np # numpy 모듈 추가
import torch # torch 모듈 추가
from flask import Flask, Response
from ultralytics import YOLO

# --- 1. 영상 수신 및 객체 탐지 파트 ---
# Load YOLOv8 model
device = 'cuda' if torch.cuda.is_available() else 'cpu' # CUDA 사용 가능 여부 확인
#model = YOLO('/home/bbang/Workspace/intel_final/intel_final_project/runs/detect/yolov8x_tomato3/weights/best.pt').to(device) # 모델을 해당 장치로 로드
model = YOLO('/home/bbang/Workspace/intel_final/intel_final_project/runs/detect/yolov8x_tomato10/weights/best.pt').to(device) # 모델을 해당 장치로 로드

BASE_DIR = Path(__file__).resolve().parent
CREDENTIAL_FILE = BASE_DIR / "idlist.txt"
AUTH_PROMPT = b"AUTH_REQUEST\n"
AUTH_OK = b"AUTH_OK\n"
AUTH_FAIL = b"AUTH_FAIL\n"

print(f"YOLOv8 모델을 {device} 장치로 로드했습니다.") # 로드된 장치 출력

# 최신 처리된 프레임을 저장할 변수
output_frame = None
lock = threading.Lock()
client_frames = {}
frames_lock = threading.Lock()


def load_credentials():
    """Load allowed id/password pairs from CREDENTIAL_FILE."""
    credentials = {}
    try:
        with CREDENTIAL_FILE.open("r", encoding="utf-8") as cred_file:
            for raw_line in cred_file:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if ":" not in line:
                    print(f"Skipping malformed credential entry: {line}")
                    continue
                user, password = line.split(":", 1)
                credentials[user.strip()] = password.strip()
    except FileNotFoundError:
        print(f"Credential file not found at {CREDENTIAL_FILE}. Rejecting new connections.")
    return credentials


def _recv_until_newline(conn):
    """Receive data until a newline is encountered or connection closes."""
    buffer = b""
    while b"\n" not in buffer and len(buffer) < 4096:
        chunk = conn.recv(1024)
        if not chunk:
            break
        buffer += chunk
    if b"\n" in buffer:
        line, remainder = buffer.split(b"\n", 1)
    else:
        line, remainder = buffer, b""
    return line, remainder


def authenticate_client(conn, addr):
    """Authenticate an incoming client connection."""
    credentials = load_credentials()
    if not credentials:
        try:
            conn.sendall(AUTH_FAIL)
        except OSError:
            pass
        print(f"접속 거부: 자격 증명 파일을 읽을 수 없어 {addr} 연결을 종료합니다.")
        return False, b"", None

    try:
        conn.sendall(AUTH_PROMPT)
        raw_line, remainder = _recv_until_newline(conn)
    except OSError as exc:
        print(f"{addr} 인증 중 소켓 오류 발생: {exc}")
        return False, b"", None

    if not raw_line:
        try:
            conn.sendall(AUTH_FAIL)
        except OSError:
            pass
        print(f"접속 거부: {addr} 로부터 인증 정보가 전달되지 않았습니다.")
        return False, b"", None

    try:
        credential_line = raw_line.decode("utf-8").strip()
    except UnicodeDecodeError:
        conn.sendall(AUTH_FAIL)
        print(f"접속 거부: {addr} 인증 정보 디코딩 실패")
        return False, b"", None

    if ":" not in credential_line:
        conn.sendall(AUTH_FAIL)
        print(f"접속 거부: {addr} 인증 정보 형식 오류")
        return False, b"", None

    user_id, supplied_password = [part.strip() for part in credential_line.split(":", 1)]
    expected_password = credentials.get(user_id)

    if expected_password and hmac.compare_digest(expected_password, supplied_password):
        try:
            conn.sendall(AUTH_OK)
        except OSError:
            return False, b""
        print(f"클라이언트 {addr} 인증 성공 (ID: {user_id})")
        return True, remainder, user_id

    conn.sendall(AUTH_FAIL)
    print(f"접속 거부: {addr} 인증 실패 (ID: {user_id})")
    return False, b"", None


def handle_client(conn, addr, client_id, prefetched_data=b""):
    """클라이언트로부터 영상을 받아 처리하는 함수"""
    global output_frame
    conf_threshold = 0.6
    print(f"클라이언트 {addr}가 연결되었습니다. (ID: {client_id})")
    
    data = prefetched_data
    payload_size = struct.calcsize(">L")

    # FPS calculation variables
    fps_start_time = time.time()
    fps_frame_count = 0
    display_fps = 0

    while True:
        try:
            # 설정한 payload_size 만큼의 데이터를 먼저 받음 (프레임 크기 정보)
            while len(data) < payload_size:
                packet = conn.recv(4*1024)
                if not packet: raise ConnectionAbortedError
                data += packet
            
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack(">L", packed_msg_size)[0]

            # 실제 프레임 데이터를 모두 수신
            while len(data) < msg_size:
                data += conn.recv(4*1024)
            
            frame_data = data[:msg_size]
            data = data[msg_size:]

            # JPEG 프레임 역직렬화
            frame_array = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)
            
            if frame is None: # 디코딩 실패 시 건너뛰기
                print("오류: JPEG 프레임 디코딩 실패")
                continue

            # Original frame dimensions
            h_orig, w_orig, _ = frame.shape

            # Dimensions for YOLO inference
            yolo_w = 640
            yolo_h = int(yolo_w * (h_orig / w_orig)) # Calculate height to preserve aspect ratio
            resized_frame_for_yolo = cv2.resize(frame, (yolo_w, yolo_h)) # This is the frame YOLO sees

            # YOLOv8 Inference on the resized frame
            results = model(resized_frame_for_yolo, verbose=False)

            # Prepare rendered_frame as a copy of the original high-res frame
            rendered_frame = frame.copy()

            # Calculate scaling factors from YOLO input size to original frame size
            scale_x = w_orig / yolo_w
            scale_y = h_orig / yolo_h

            # Manually draw bounding boxes using normalized coordinates
            for box in results[0].boxes:
                conf = box.conf[0]
                # 신뢰도 임계값 체크
                if conf < conf_threshold:
                    continue

                # Get normalized coordinates (xyxyn) relative to resized_frame_for_yolo
                nx1, ny1, nx2, ny2 = box.xyxyn[0]
                
                # Convert to pixel coordinates relative to resized_frame_for_yolo
                # Then scale them to the original frame's dimensions
                x1 = int(nx1 * yolo_w * scale_x)
                y1 = int(ny1 * yolo_h * scale_y)
                x2 = int(nx2 * yolo_w * scale_x)
                y2 = int(ny2 * yolo_h * scale_y)
                
                cls = int(box.cls[0])
                class_name = model.names[cls]
                label = f"{class_name} {conf:.2f}"

                # 'ripe' 또는 'rotten'이 감지되면 클라이언트로 좌표 전송
                if class_name in ['ripe', 'rotten']:
                    try:
                        message = f"{class_name}:{x1},{y1}"
                        conn.sendall(message.encode('utf-8'))
                    except socket.error as e:
                        print(f"클라이언트로 메시지 전송 실패: {e}")
                
                cv2.rectangle(rendered_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # Scale font size for the label
                label_font_scale = 0.5 * (w_orig / yolo_w)
                cv2.putText(rendered_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, label_font_scale, (0, 255, 0), 2)

            # FPS calculation
            fps_frame_count += 1
            if time.time() - fps_start_time >= 1.0:
                display_fps = fps_frame_count / (time.time() - fps_start_time)
                fps_frame_count = 0
                fps_start_time = time.time()

            # Display FPS on the frame
            # Scale font size for FPS display
            fps_font_scale = 0.7 * (w_orig / yolo_w)
            cv2.putText(rendered_frame, f"FPS: {display_fps:.2f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, fps_font_scale, (0, 0, 255), 2)

            # 처리된 프레임을 공유 변수에 저장
            with lock:
                output_frame = rendered_frame.copy()
            with frames_lock:
                client_frames[client_id] = rendered_frame.copy()
        except (ConnectionAbortedError, ConnectionResetError, struct.error): # pickle.UnpicklingError 제거
            break


    print(f"클라이언트 {addr} 연결이 끊겼습니다.")
    with frames_lock:
        client_frames.pop(client_id, None)
    conn.close()

def start_socket_server():
    """여러 라즈베리 파이의 연결을 기다리는 소켓 서버"""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 9999))
    server_socket.listen(10)
    print("소켓 서버가 시작되었습니다. 클라이언트의 연결을 기다립니다...")

    while True:
        conn, addr = server_socket.accept()
        is_authenticated, leftover, client_id = authenticate_client(conn, addr)
        if not is_authenticated:
            conn.close()
            continue

        # 클라이언트마다 새로운 스레드를 생성하여 처리
        thread = threading.Thread(target=handle_client, args=(conn, addr, client_id, leftover))
        thread.daemon = True
        thread.start()

# --- 2. Flask 웹 스트리밍 파트 ---
app = Flask(__name__)

def generate_frames(client_id):
    """요청한 클라이언트 ID에 해당하는 프레임을 스트리밍."""
    while True:
        with frames_lock:
            frame = client_frames.get(client_id)
        if frame is None:
            time.sleep(0.05)
            continue

        flag, encoded_image = cv2.imencode(".jpg", frame)
        if not flag:
            time.sleep(0.05)
            continue

        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encoded_image) + b'\r\n')


@app.route("/video_feed/<client_id>")
def video_feed(client_id):
    return Response(generate_frames(client_id),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/")
def index():
    # 간단한 웹페이지. templates/index.html 파일을 만들어야 합니다.
    with frames_lock:
        active_clients = ", ".join(sorted(client_frames)) or "없음"
    return (
        "<h1>토마토 감지 가뵤자고~</h1>"
        "<p>/video_feed/&lt;client_id&gt; 형식으로 접속하세요.</p>"
        f"<p>현재 연결된 클라이언트: {active_clients}</p>"
    )


if __name__ == '__main__':
    # 소켓 서버를 별도의 스레드에서 실행
    socket_thread = threading.Thread(target=start_socket_server)
    socket_thread.daemon = True
    socket_thread.start()

    # Flask 앱 실행
    app.run(host='0.0.0.0', port=5000, debug=False)

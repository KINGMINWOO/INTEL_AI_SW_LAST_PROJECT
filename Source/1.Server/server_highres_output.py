# server_highres_output.py (컴퓨터에서 실행)
import math
import socket
import cv2
# import pickle # pickle은 더 이상 사용하지 않음
import struct
import threading
import time
import hmac
from pathlib import Path
from typing import Optional

import numpy as np # numpy 모듈 추가
import torch # torch 모듈 추가
from flask import Flask, Response
from ultralytics import YOLO

try:
    from nav2_bridge import Nav2Bridge, Nav2Result, create_nav2_bridge
except Exception as nav2_import_error:  # noqa: BLE001
    Nav2Bridge = None  # type: ignore
    Nav2Result = None  # type: ignore
    create_nav2_bridge = None  # type: ignore
    print(f"Nav2 브리지 모듈을 불러오지 못했습니다: {nav2_import_error}")

DETECTION_WINDOW_SECONDS = 3.0
DETECTION_COUNT_THRESHOLD = 5
ABS_CENTER_THRESHOLD = 0.05

# --- 1. 영상 수신 및 객체 탐지 파트 ---
# YOLOv8 모델 로드
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
robot_clients = {}
robot_states = {}
control_lock = threading.Lock()
nav2_bridge: Optional["Nav2Bridge"] = None
nav2_motion_lock = threading.Lock()
nav2_motion_thread: Optional[threading.Thread] = None
nav2_active_robot: Optional[str] = None


def cancel_nav2_navigation(robot_id: Optional[str] = None, wait: bool = True) -> bool:
    """현재 Nav2 목표가 진행 중이라면 취소."""
    global nav2_motion_thread, nav2_active_robot
    cancelled = False

    target_robot = robot_id.upper() if robot_id else nav2_active_robot

    if nav2_bridge is not None:
        try:
            cancelled = nav2_bridge.cancel_current_goal()
            if cancelled:
                print("Nav2 목표 취소 요청을 완료했습니다.")
        except Exception as exc:  # noqa: BLE001
            print(f"Nav2 목표 취소 중 오류: {exc}")

    if wait:
        thread: Optional[threading.Thread]
        with nav2_motion_lock:
            thread = nav2_motion_thread
        if thread and thread.is_alive():
            thread.join(timeout=2.0)
        with nav2_motion_lock:
            if nav2_motion_thread and (nav2_motion_thread is thread) and (thread is not None) and (not thread.is_alive()):
                nav2_motion_thread = None

    if target_robot:
        with control_lock:
            state = robot_states.get(target_robot)
            if state is not None:
                if state.get("nav2_active") and not state.get("nav2_paused_label"):
                    state["nav2_paused_label"] = state.get("nav2_label")
                    state["nav2_paused_goal"] = state.get("nav2_goal")
                    state["nav2_paused_yaw"] = state.get("nav2_goal_yaw", 0.0)
                state["nav2_active"] = False
                if not state.get("nav2_paused_label"):
                    state["nav2_label"] = None
                    state["nav2_goal"] = None
                    state["nav2_goal_yaw"] = 0.0
        if nav2_active_robot == target_robot:
            nav2_active_robot = None

    return cancelled


def schedule_next_nav2(robot_id: str) -> None:
    upper_id = robot_id.upper()
    next_step = None
    with control_lock:
        state = robot_states.get(upper_id)
        if state is None:
            return
        if state.get("nav2_active"):
            return
        queue = state.setdefault("nav2_queue", [])
        if queue:
            next_step = queue.pop(0)
    if next_step:
        command, goal, yaw = next_step
        success, reason = dispatch_move_command(upper_id, command=command, goal=goal, yaw=yaw)
        if not success:
            print(f"{upper_id} 큐 명령 실행 실패: {reason}")


def enqueue_nav2_sequence(robot_id: str, sequence: list[tuple[str, tuple[float, float], float]]) -> None:
    upper_id = robot_id.upper()
    with control_lock:
        state = robot_states.setdefault(
            upper_id,
            {
                "stop_sent": False,
                "pose": (0.0, 0.0),
                "nav2_label": None,
                "nav2_goal": None,
                "nav2_goal_yaw": 0.0,
                "nav2_paused_label": None,
                "nav2_paused_goal": None,
                "nav2_paused_yaw": None,
                "nav2_active": False,
                "detection_history": {"ripe": [], "rotten": []},
                "nav2_queue": [],
            },
        )
        state["nav2_queue"] = list(sequence)
        state["nav2_paused_label"] = None
        state["nav2_paused_goal"] = None
        state["nav2_paused_yaw"] = None
    schedule_next_nav2(upper_id)


def start_nav2_navigation(
    robot_id: str,
    goal_point: tuple[float, float],
    label: Optional[str],
    yaw: Optional[float],
) -> tuple[bool, str]:
    """Nav2 이동을 비동기로 시작."""
    global nav2_motion_thread, nav2_active_robot

    if nav2_bridge is None or Nav2Result is None:
        return False, "NAV2_UNAVAILABLE"

    cancel_nav2_navigation(wait=True)

    upper_id = robot_id.upper()
    yaw_value = float(yaw) if yaw is not None else 0.0

    with control_lock:
        state = robot_states.setdefault(
            upper_id,
            {
                "stop_sent": False,
                "pose": (0.0, 0.0),
                "nav2_label": None,
                "nav2_goal": None,
                "nav2_goal_yaw": 0.0,
                "nav2_paused_label": None,
                "nav2_paused_goal": None,
                "nav2_paused_yaw": None,
                "nav2_active": False,
                "detection_history": {"ripe": [], "rotten": []},
                "nav2_queue": [],
            },
        )
        state["nav2_active"] = True
        state["nav2_label"] = label
        state["nav2_goal"] = goal_point
        state["nav2_goal_yaw"] = yaw_value
        state["nav2_paused_label"] = None
        state["nav2_paused_goal"] = None
        state["nav2_paused_yaw"] = None
        state["stop_sent"] = False
        history = state.setdefault("detection_history", {"ripe": [], "rotten": []})
        for buf in history.values():
            buf.clear()

    nav2_active_robot = upper_id

    def _worker() -> None:
        global nav2_motion_thread, nav2_active_robot
        result: Optional[Nav2Result] = None
        try:
            result = nav2_bridge.navigate(
                goal_point[0],
                goal_point[1],
                yaw=yaw_value,
            )
            print(f"Nav2 목표 ({goal_point[0]:.2f}, {goal_point[1]:.2f}) 결과: {result.message}")
        except Exception as exc:  # noqa: BLE001
            print(f"Nav2 이동 중 예외 발생: {exc}")
            result = Nav2Result(False, -1, str(exc))
        finally:
            with control_lock:
                state = robot_states.get(upper_id)
                if state is not None:
                    state["nav2_active"] = False
                    if not state.get("nav2_paused_label"):
                        state["nav2_label"] = None
                        state["nav2_goal"] = None
                        state["nav2_goal_yaw"] = 0.0
            nav2_active_robot = None
            with nav2_motion_lock:
                if nav2_motion_thread is threading.current_thread():
                    nav2_motion_thread = None
            if result and result.success:
                schedule_next_nav2(upper_id)

    worker = threading.Thread(target=_worker, daemon=True)
    with nav2_motion_lock:
        nav2_motion_thread = worker
    worker.start()
    return True, "NAV2_STARTED"


def send_stop_to_robot(target_id: str) -> None:
    upper_id = target_id.upper()
    with control_lock:
        target_conn = robot_clients.get(upper_id)
    if not target_conn:
        return
    try:
        target_conn.sendall(b"STOP\n")
        print(f"{upper_id} 에게 STOP 명령 전송.")
        with control_lock:
            state = robot_states.get(upper_id)
            if state is not None:
                state["stop_sent"] = True
    except OSError as exc:  # noqa: BLE001
        print(f"{upper_id} STOP 명령 전송 실패: {exc}")


def should_trigger_auto_stop(robot_id: str, category: str, x_center_norm: float) -> bool:
    now = time.monotonic()
    upper_id = robot_id.upper()
    expected_label = f"{category}@go"

    with control_lock:
        state = robot_states.get(upper_id)
        if state is None:
            return False
        if abs(x_center_norm - 0.5) > ABS_CENTER_THRESHOLD:
            return False
        if not state.get("nav2_active"):
            return False
        if nav2_active_robot and nav2_active_robot != upper_id:
            return False
        if state.get("nav2_label") != expected_label:
            return False
        history = state.setdefault("detection_history", {"ripe": [], "rotten": []})
        buffer = history.setdefault(category, [])
        buffer.append(now)
        while buffer and now - buffer[0] > DETECTION_WINDOW_SECONDS:
            buffer.pop(0)
        if len(buffer) >= DETECTION_COUNT_THRESHOLD:
            buffer.clear()
            state["nav2_paused_label"] = expected_label
            state["nav2_paused_goal"] = state.get("nav2_goal")
            state["nav2_paused_yaw"] = state.get("nav2_goal_yaw", 0.0)
            state["nav2_label"] = None
            state["nav2_active"] = False
            return True
    return False


def resume_nav2_for_robot(robot_id: str) -> bool:
    upper_id = robot_id.upper()
    with control_lock:
        state = robot_states.get(upper_id)
        if state is None:
            return False
        label = state.get("nav2_paused_label")
        goal = state.get("nav2_paused_goal")
        yaw = state.get("nav2_paused_yaw")
        if not label or goal is None:
            return False
        state["nav2_paused_label"] = None
        state["nav2_paused_goal"] = None
        state["nav2_paused_yaw"] = None

    success, status = dispatch_move_command(upper_id, command=label, goal=goal, yaw=yaw)
    if success:
        print(f"{upper_id} Nav2 재개: {label} → ({goal[0]:.2f}, {goal[1]:.2f})")
    else:
        print(f"{upper_id} Nav2 재개 실패: {status}")
    return success


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
    is_robot = client_id.upper().startswith("TURTLE")
    if is_robot:
        with control_lock:
            robot_clients[client_id.upper()] = conn
            robot_states.setdefault(
                client_id.upper(),
                {
                    "stop_sent": False,
                    "pose": (0.0, 0.0),
                    "nav2_label": None,
                    "nav2_goal": None,
                    "nav2_goal_yaw": 0.0,
                    "nav2_paused_label": None,
                    "nav2_paused_goal": None,
                    "nav2_paused_yaw": None,
                    "nav2_active": False,
                    "detection_history": {"ripe": [], "rotten": []},
                    "nav2_queue": [],
                },
            )

    data = prefetched_data
    payload_size = struct.calcsize(">L")

    # FPS 계산용 변수
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

            if msg_size == 0:
                while b"\n" not in data:
                    chunk = conn.recv(1024)
                    if not chunk:
                        raise ConnectionAbortedError
                    data += chunk
                raw, _, remainder = data.partition(b"\n")
                data = remainder
                control_line = raw.decode("utf-8", errors="ignore").strip()
                if not control_line:
                    continue
                if control_line.startswith("POSE:") and is_robot:
                    try:
                        _, pose_payload = control_line.split(":", 1)
                        x_str, y_str = pose_payload.split(",", 1)
                        pose = (float(x_str), float(y_str))
                    except ValueError:
                        print(f"{client_id} 잘못된 POSE 메시지: {control_line}")
                        continue
                    with control_lock:
                        state = robot_states.setdefault(
                            client_id.upper(),
                            {
                                "stop_sent": False,
                                "pose": pose,
                                "nav2_label": None,
                                "nav2_goal": None,
                                "nav2_goal_yaw": 0.0,
                                "nav2_paused_label": None,
                                "nav2_paused_goal": None,
                                "nav2_paused_yaw": None,
                                "nav2_active": False,
                                "detection_history": {"ripe": [], "rotten": []},
                                "nav2_queue": [],
                            },
                        )
                        state["pose"] = pose
                    continue
                if control_line.lower() == "robot@done" and is_robot:
                    resume_nav2_for_robot(client_id.upper())
                    continue
                else:
                    print(f"{client_id} 제어 메시지 수신: {control_line}")
                    continue

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

            # 원본 프레임 해상도
            h_orig, w_orig, _ = frame.shape

            # YOLO 추론용 프레임 크기 설정
            yolo_w = 640
            yolo_h = int(yolo_w * (h_orig / w_orig)) # 종횡비를 유지한 높이 계산
            resized_frame_for_yolo = cv2.resize(frame, (yolo_w, yolo_h)) # YOLO 입력용 프레임

            # YOLOv8 추론 수행
            results = model(resized_frame_for_yolo, verbose=False)
            
            # 원본 해상도의 프레임 복사본 준비
            rendered_frame = frame.copy()
            
            # YOLO 입력 크기 대비 원본 크기 스케일 계산
            scale_x = w_orig / yolo_w
            scale_y = h_orig / yolo_h
            
            # 정규화 좌표를 이용해 바운딩 박스 그리기
            for box in results[0].boxes:
                conf = box.conf[0]
                # 신뢰도 임계값 체크
                if conf < conf_threshold:
                    continue

                # YOLO 정규화 좌표 읽기
                nx1, ny1, nx2, ny2 = box.xyxyn[0]
                
                # 픽셀 좌표로 변환 후 원본 크기에 맞게 스케일링
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
                        message = f"{class_name}:{x1},{y1}\n"
                        conn.sendall(message.encode('utf-8'))
                    except socket.error as e:
                        print(f"클라이언트로 메시지 전송 실패: {e}")

                if is_robot and class_name in {"ripe", "rotten"}:
                    category = "ripe" if class_name == "ripe" else "rotten"
                    if should_trigger_auto_stop(client_id.upper(), category, ((x1 + x2) / 2) / w_orig):
                        cancel_nav2_navigation(client_id.upper())
                        send_stop_to_robot(client_id)
                        y_center_norm = ((y1 + y2) / 2) / h_orig
                        if y_center_norm < 1.0 / 3:
                            arm_topic = b"robot@high\n"
                        elif y_center_norm < 2.0 / 3:
                            arm_topic = b"robot@middle\n"
                        else:
                            arm_topic = b"robot@low\n"
                        try:
                            conn.sendall(arm_topic)
                            print(f"{client_id} 로봇팔 명령 전송: {arm_topic.decode().strip()}")
                        except socket.error as exc:
                            print(f"{client_id} 로봇팔 명령 전송 실패: {exc}")

                cv2.rectangle(rendered_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 라벨 표시용 글꼴 크기 조정
                label_font_scale = 0.5 * (w_orig / yolo_w)
                cv2.putText(rendered_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, label_font_scale, (0, 255, 0), 2)
            
            # FPS 계산
            fps_frame_count += 1
            if time.time() - fps_start_time >= 1.0:
                display_fps = fps_frame_count / (time.time() - fps_start_time)
                fps_frame_count = 0
                fps_start_time = time.time()
            
            # FPS 표시 (글꼴 크기 포함)
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
    if is_robot:
        with control_lock:
            robot_clients.pop(client_id.upper(), None)
            robot_states.pop(client_id.upper(), None)
    conn.close()


def dispatch_move_command(
    target_id: str,
    command: Optional[str] = None,
    origin: tuple[float, float] = (0.0, 0.0),
    goal: tuple[float, float] = (0.0, 1.5),
    yaw: Optional[float] = None,
):
    """TURTLEBOT 클라이언트로 이동 명령을 전송하거나 Nav2에 목표를 전달."""
    global nav2_bridge

    upper_id = target_id.upper()

    if nav2_bridge is not None and Nav2Result is not None:
        success, status = start_nav2_navigation(upper_id, goal, command, yaw)
        return success, status

    with control_lock:
        target_conn = robot_clients.get(upper_id)
        pose = None
        if target_conn:
            state = robot_states.setdefault(
                upper_id,
                {
                    "stop_sent": False,
                    "pose": (0.0, 0.0),
                    "nav2_label": None,
                    "nav2_goal": None,
                    "nav2_goal_yaw": 0.0,
                    "nav2_paused_label": None,
                    "nav2_paused_goal": None,
                    "nav2_paused_yaw": None,
                    "nav2_active": False,
                    "detection_history": {"ripe": [], "rotten": []},
                    "nav2_queue": [],
                },
            )
            state["stop_sent"] = False
            state["nav2_label"] = command
            state["nav2_goal"] = goal
            state["nav2_goal_yaw"] = float(yaw) if yaw is not None else 0.0
            state["nav2_paused_label"] = None
            state["nav2_paused_goal"] = None
            state["nav2_paused_yaw"] = None
            pose = state.get("pose")
    if target_conn is None:
        return False, "TARGET_NOT_CONNECTED"
    start = pose if pose is not None else origin
    message = f"MOVE:{start[0]:.2f},{start[1]:.2f}->{goal[0]:.2f},{goal[1]:.2f}\n"
    try:
        target_conn.sendall(message.encode("utf-8"))
        print(f"{target_id} 에게 명령 전송: {message.strip()}")
        return True, "OK"
    except OSError as exc:
        print(f"{target_id} 명령 전송 실패: {exc}")
        return False, "SEND_FAILED"


def handle_user_client(conn, addr, client_id, prefetched_data=b""):
    """USERxx 명령을 받아 대응되는 TURTLEBOTxx로 전달."""
    print(f"사용자 채널 {addr} 접속 (ID: {client_id})")
    suffix = client_id.upper()[4:] if len(client_id) >= 4 else ""
    target_id = f"TURTLE{suffix}" if suffix else None
    buffer = bytearray(prefetched_data)
    try:
        while True:
            if b"\n" not in buffer:
                chunk = conn.recv(1024)
                if not chunk:
                    break
                buffer.extend(chunk)
                continue

            raw, _, remainder = buffer.partition(b"\n")
            buffer = bytearray(remainder)
            message = raw.decode("utf-8", errors="ignore").strip()
            if not message:
                continue
            print(f"[{client_id}] 수신: {message}")
            command = message.lower()
            goal_map = {
                "ripe@go": ((1.5, -0.5), math.pi * 0.5),
                "ripe@done": ((1.5, 0.5), math.pi),
                "rotten@go": ((1.5, -0.5), math.pi * 0.5),
                "rotten@done": ((-1.5, 0.5), math.pi * 1.5),
                "dump@done": ((-1.5, -0.5), 0.0),
            }
            nav2_queue_templates = {
                "turtle@go": [
                    "ripe@go",
                    "ripe@done",
                    "dump@done",
                    "rotten@go",
                    "rotten@done",
                    "dump@done",
                ]
            }

            if command in goal_map:
                if not target_id:
                    conn.sendall(b"ERROR:TARGET_UNKNOWN\n")
                    continue
                goal, yaw = goal_map[command]
                success, reason = dispatch_move_command(target_id, command=command, goal=goal, yaw=yaw)
                response = b"CMD_OK\n" if success else f"ERROR:{reason}\n".encode("utf-8")
                conn.sendall(response)
            elif command in nav2_queue_templates:
                if not target_id:
                    conn.sendall(b"ERROR:TARGET_UNKNOWN\n")
                    continue
                cancel_nav2_navigation(target_id, wait=True)
                sequence = []
                for step in nav2_queue_templates[command]:
                    goal, yaw = goal_map.get(step, ((0.0, 0.0), 0.0))
                    sequence.append((step, goal, yaw))
                enqueue_nav2_sequence(target_id, sequence)
                conn.sendall(b"CMD_OK\n")
            elif command == "stop":
                cancelled = cancel_nav2_navigation(target_id)
                if target_id:
                    send_stop_to_robot(target_id)
                print(f"[{client_id}] STOP 명령 처리: Nav2 취소={'성공' if cancelled else '없음'}")
                response = b"CMD_OK\n" if cancelled else b"CMD_OK\n"
                conn.sendall(response)
            else:
                conn.sendall(b"ERROR:UNKNOWN_CMD\n")
    except OSError as exc:
        print(f"사용자 채널 오류 ({client_id}): {exc}")
    finally:
        print(f"사용자 채널 종료 (ID: {client_id})")
        conn.close()

def start_socket_server():
    """여러 라즈베리 파이의 연결을 기다리는 소켓 서버"""
    global nav2_bridge

    if nav2_bridge is None and create_nav2_bridge is not None:
        try:
            nav2_bridge = create_nav2_bridge()
            print("Nav2 브리지를 초기화했습니다.")
        except Exception as exc:  # noqa: BLE001
            print(f"Nav2 브리지 초기화 실패: {exc}")

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

        if client_id.upper().startswith("USER"):
            thread = threading.Thread(target=handle_user_client, args=(conn, addr, client_id, leftover))
        else:
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

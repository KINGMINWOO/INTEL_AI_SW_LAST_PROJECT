# server_highres_output.py (컴퓨터에서 실행)
import math
import os
import socket
import cv2
# import pickle # pickle은 더 이상 사용하지 않음
import struct
import subprocess
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

try:
    from farm_storage import FarmDataLogger
except Exception as farm_storage_import_error:  # noqa: BLE001
    FarmDataLogger = None  # type: ignore
    print(f"농가 데이터 로거를 불러오지 못했습니다: {farm_storage_import_error}")

DETECTION_WINDOW_SECONDS = 3.0
DETECTION_COUNT_THRESHOLD = 1
ABS_CENTER_THRESHOLD = 0.05
POSE_LOG_INTERVAL_SECONDS = 5.0
YAW_ALIGNMENT_THRESHOLD = 0.01

COMMAND_SPECS: dict[str, dict[str, Optional[float] | tuple[float, float] | str]] = {
    "ripe@go": {"mode": "manual_offset", "target": (2.0, 0.0), "yaw": None},
    "ripe@done": {"mode": "nav2", "target": (1.817, 0.786), "yaw": 3.117},
    "rotten@go": {"mode": "manual_offset", "target": (1.5, -0.5), "yaw": None},
    "rotten@done": {"mode": "nav2", "target": (0.1, 0.8), "yaw": -0.263},
    "check2@point": {"mode": "nav2", "target": (0.8, 0.45), "yaw": -1.524},
    "check1point": {"mode": "nav2", "target": (1.5, 0.45), "yaw": math.pi},
    "dump@done": {"mode": "nav2", "target": (0.0, 0.0), "yaw": 0.0},
    "dump1@done": {"mode": "nav2", "target": (0.0, 0.0), "yaw": 0.0},
    "dump2@done": {"mode": "nav2", "target": (0.0, 0.0), "yaw": 0.0},
}

NAV2_QUEUE_TEMPLATES: dict[str, list[str]] = {
    "turtle@go": [
        "ripe@go",
        #"check@point",
        "ripe@done",
        "check1@point",
        "dump1@done",
        "rotten@go",
        #"check@point",
        "rotten@done",
        "check2@point",
        "dump2@done",
    ],
    "turtle2@go": [
        "rotten@go",
        "rotten@done",
        "check@point",
        "dump2@done",
    ]
}

# --- 1. 영상 수신 및 객체 탐지 파트 ---
# YOLOv8 모델 로드
device = 'cuda' if torch.cuda.is_available() else 'cpu' # CUDA 사용 가능 여부 확인
#model = YOLO('/home/bbang/Workspace/intel_final/intel_final_project/runs/detect/yolov8x_tomato3/weights/best.pt').to(device) # 모델을 해당 장치로 로드
#model = YOLO('/home/bbang/Workspace/intel_final/intel_final_project/runs/detect/yolov8x_tomato10/weights/best.pt').to(device) # 모델을 해당 장치로 로드
model = YOLO('best.pt').to(device)
#model = YOLO('/home/bbang/Workspace/intel_final/intel_final_project/runs/detect/yolov8x_final3/weights/best.pt').to(device)

model_lock = threading.Lock()

BASE_DIR = Path(__file__).resolve().parent
CREDENTIAL_FILE = BASE_DIR / "idlist.txt"
AUTH_PROMPT = b"AUTH_REQUEST\n"
AUTH_OK = b"AUTH_OK\n"
AUTH_FAIL = b"AUTH_FAIL\n"
LINE_TRACER_SCRIPT = (BASE_DIR.parent / "3.Robot" / "1.ROS" / "line_tracer_remote.py").resolve()
LINE_TRACER_STOP_TIMEOUT = 2.0

DUMP_STAGE_TARGET = (-0.166, -0.227)
DUMP_STAGE_YAW = -2.359
DUMP_FINAL_QUEUE_LABEL = "__dump@done_finalize__"
LINE_TRACER_LABELS = {"ripe@go", "rotten@go"}

print(f"YOLOv8 모델을 {device} 장치로 로드했습니다.") # 로드된 장치 출력

# 최신 처리된 프레임을 저장할 변수
output_frame = None
lock = threading.Lock()
client_frames = {}
frames_lock = threading.Lock()
robot_clients = {}
robot_states = {}
control_lock = threading.Lock()
cctv_clients = {}
nav2_bridge: Optional["Nav2Bridge"] = None
nav2_motion_lock = threading.Lock()
nav2_motion_thread: Optional[threading.Thread] = None
nav2_active_robot: Optional[str] = None
alignment_threads: dict[str, threading.Thread] = {}
DEFAULT_FARM_DB_URL = "mysql+pymysql://user01:user1234@10.10.16.29:3306/smart_farm"
#DEFAULT_FARM_DB_URL = "mysql+pymysql://user01:user1234@192.168.0.4:3306/smart_farm"
farm_db_url = os.getenv("FARM_DB_URL") or DEFAULT_FARM_DB_URL
if FarmDataLogger:
    if "your_password" in farm_db_url:
        print("경고: FARM_DB_URL이 설정되지 않았거나 기본 비밀번호가 그대로입니다. 실제 MariaDB 비밀번호로 수정하세요.")
    farm_logger = FarmDataLogger(farm_db_url)
else:
    farm_logger = None


def _parse_float(token: str) -> Optional[float]:
    try:
        return float(token)
    except (TypeError, ValueError):
        return None


def _parse_int(token: str) -> Optional[int]:
    try:
        return int(float(token))
    except (TypeError, ValueError):
        return None


def try_handle_sensor_payload(source_id: str, payload: str) -> bool:
    if not payload:
        return False

    tokens = [segment.strip() for segment in payload.split("@")]
    if not tokens:
        return False

    kind = tokens[0].lower()
    values = tokens[1:]
    device_id = source_id.upper()

    if kind == "air":
        if not values:
            print(f"{device_id} 잘못된 공기 센서 페이로드: {payload}")
            return True

        temperature = _parse_float(values[0]) if len(values) >= 1 else None
        humidity = _parse_float(values[1]) if len(values) >= 2 else None
        air_quality = _parse_int(values[2]) if len(values) >= 3 else None
        illuminance = _parse_int(values[3]) if len(values) >= 4 else None

        if farm_logger:
            stored = farm_logger.log_air_sample(
                device_id=device_id,
                temperature_c=temperature,
                humidity_pct=humidity,
                air_quality=air_quality,
                illuminance_lux=illuminance,
            )
            status = "저장" if stored else "로컬로그"
        else:
            status = "미초기화"
        print(
            f"[FARM-AIR] {device_id} temp={temperature} hum={humidity} "
            f"air={air_quality} lux={illuminance} ({status})"
        )
        return True

    if kind == "land":
        if not values:
            print(f"{device_id} 잘못된 토양 센서 페이로드: {payload}")
            return True

        temperature = _parse_float(values[0]) if len(values) >= 1 else None
        humidity = _parse_float(values[1]) if len(values) >= 2 else None
        ec = _parse_int(values[2]) if len(values) >= 3 else None
        ph = _parse_float(values[3]) if len(values) >= 4 else None

        if farm_logger:
            stored = farm_logger.log_land_sample(
                device_id=device_id,
                temperature_c=temperature,
                humidity_pct=humidity,
                ec=ec,
                ph=ph,
            )
            status = "저장" if stored else "로컬로그"
        else:
            status = "미초기화"
        print(
            f"[FARM-LAND] {device_id} temp={temperature} hum={humidity} "
            f"ec={ec} ph={ph} ({status})"
        )
        return True

    return False


line_tracer_lock = threading.Lock()
line_tracer_processes: dict[str, subprocess.Popen] = {}


def handle_line_tracer_exit(robot_id: str, return_code: Optional[int]) -> None:
    """라인 트레이서 프로세스 종료 후 로봇 주행 상태를 정리."""
    upper_id = robot_id.upper()
    should_stop = False
    manual_label: Optional[str] = None
    is_paused = False

    with control_lock:
        state = robot_states.get(upper_id)
        if state is not None:
            if state.get("nav2_paused_label") is not None:
                is_paused = True

            manual_active = bool(state.get("nav2_manual_active"))
            nav_label = state.get("nav2_label")
            paused_label = state.get("nav2_paused_label")
            stop_sent = bool(state.get("stop_sent"))
            if nav_label in LINE_TRACER_LABELS:
                manual_label = nav_label
            elif paused_label in LINE_TRACER_LABELS:
                manual_label = paused_label
                state["nav2_label"] = paused_label
                state["nav2_paused_label"] = None
                state["nav2_paused_goal"] = None
                state["nav2_paused_yaw"] = None
            if manual_active and nav_label in LINE_TRACER_LABELS and not stop_sent:
                state["stop_sent"] = True
                should_stop = True
    if should_stop:
        print(f"{upper_id} 라인 트레이서 종료 감지(exit={return_code}) → STOP 명령 전송.")
        send_stop_to_robot(upper_id)
    else:
        print(f"{upper_id} 라인 트레이서 종료 감지(exit={return_code}).")

    if manual_label in LINE_TRACER_LABELS:
        if is_paused:
            print(f"{upper_id} 라인 트레이서가 일시 중지되었습니다. 로봇팔의 동작을 기다립니다.")
            with control_lock:
                state = robot_states.get(upper_id)
                if state is not None:
                    state["nav2_manual_active"] = False
                    state["nav2_active"] = False
                    state["stop_sent"] = False
            stop_line_tracer(upper_id, wait_timeout=0.1)
        else:
            print(f"{upper_id} 라인 트레이서 이동 완료: 큐 다음 단계로 진행")
            with control_lock:
                state = robot_states.get(upper_id)
                if state is not None:
                    state["nav2_manual_active"] = False
                    state["nav2_active"] = False
                    state["nav2_label"] = None
                    state["nav2_goal"] = None
                    state["nav2_goal_yaw"] = 0.0
                    state["stop_sent"] = False
                    history = state.setdefault("detection_history", {"ripe": [], "rotten": []})
                    for buf in history.values():
                        buf.clear()
                    state["line_tracer_resume"] = None
            stop_line_tracer(upper_id)
            schedule_next_nav2(upper_id)
    else:
        if not is_paused:
            schedule_next_nav2(upper_id)


def normalize_angle(angle: float) -> float:
    """-pi~pi 범위의 각도로 정규화."""
    wrapped = (angle + math.pi) % (2 * math.pi) - math.pi
    return wrapped


def cancel_nav2_navigation(
    robot_id: Optional[str] = None,
    wait: bool = True,
    stop_line_tracer_on_cancel: bool = True,
) -> bool:
    """현재 Nav2 목표가 진행 중이라면 취소."""
    global nav2_motion_thread, nav2_active_robot
    cancelled = False

    target_robot = robot_id.upper() if robot_id else nav2_active_robot
    should_stop_line_tracer = False

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
                if state.get("align_pending"):
                    state["align_pending"] = False
                if state.get("nav2_active") and not state.get("nav2_paused_label"):
                    state["nav2_paused_label"] = state.get("nav2_label")
                    state["nav2_paused_goal"] = state.get("nav2_goal")
                    state["nav2_paused_yaw"] = state.get("nav2_goal_yaw", 0.0)
                state["nav2_active"] = False
                active_label = state.get("nav2_label")
                paused_label = state.get("nav2_paused_label")
                if (active_label in LINE_TRACER_LABELS) or (paused_label in LINE_TRACER_LABELS):
                    should_stop_line_tracer = True
                if not state.get("nav2_paused_label"):
                    state["nav2_label"] = None
                    state["nav2_goal"] = None
                    state["nav2_goal_yaw"] = 0.0
        if nav2_active_robot == target_robot:
            nav2_active_robot = None

    if should_stop_line_tracer and target_robot and stop_line_tracer_on_cancel:
        stop_line_tracer(target_robot)

    return cancelled


def stop_line_tracer(robot_id: str, wait_timeout: float = LINE_TRACER_STOP_TIMEOUT) -> None:
    """라인 트레이서 프로세스를 종료."""
    upper_id = robot_id.upper()
    process: Optional[subprocess.Popen]
    with line_tracer_lock:
        process = line_tracer_processes.pop(upper_id, None)
    if not process:
        return
    if process.poll() is not None:
        return
    try:
        process.terminate()
        try:
            process.wait(timeout=wait_timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=wait_timeout)
        print(f"{upper_id} 라인 트레이서를 종료했습니다.")
    except Exception as exc:  # noqa: BLE001
        print(f"{upper_id} 라인 트레이서 종료 실패: {exc}")
    finally:
        try:
            if getattr(process, "stdout", None):
                process.stdout.close()
        except Exception:  # noqa: BLE001
            pass


def start_line_tracer(robot_id: str) -> bool:
    """라인 트레이서 프로세스를 실행."""
    upper_id = robot_id.upper()
    if not LINE_TRACER_SCRIPT.exists():
        print(f"{upper_id} 라인 트레이서 스크립트를 찾을 수 없습니다: {LINE_TRACER_SCRIPT}")
        return False
    # 이전 인스턴스가 남아있다면 종료 후 재시작
    stop_line_tracer(upper_id)
    try:
        process = subprocess.Popen(
            ["python3", str(LINE_TRACER_SCRIPT)],
            cwd=str(LINE_TRACER_SCRIPT.parent),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"{upper_id} 라인 트레이서 실행 실패: {exc}")
        return False
    with line_tracer_lock:
        line_tracer_processes[upper_id] = process
    def _read_output() -> None:
        if process.stdout is None:
            return
        try:
            for raw_line in process.stdout:
                line = raw_line.rstrip("\n")
                if not line:
                    continue
                print(f"[{upper_id} LINE] {line}")
                if "라인 트레이서 종료 신호 수신" in line:
                    print(f"{upper_id} 라인 트레이서 종료 신호 감지. 프로세스를 강제 종료합니다.")
                    stop_line_tracer(upper_id)
                    break
        except Exception as exc:  # noqa: BLE001
            print(f"{upper_id} 라인 트레이서 출력 수집 중 오류: {exc}")
        finally:
            try:
                process.stdout.close()
            except Exception:  # noqa: BLE001
                pass
    def _monitor() -> None:
        return_code: Optional[int] = None
        try:
            return_code = process.wait()
        except Exception as exc:  # noqa: BLE001
            print(f"{upper_id} 라인 트레이서 모니터링 오류: {exc}")
        finally:
            with line_tracer_lock:
                existing = line_tracer_processes.get(upper_id)
                if existing is process:
                    line_tracer_processes.pop(upper_id, None)
            handle_line_tracer_exit(upper_id, return_code)
    threading.Thread(target=_read_output, daemon=True).start()
    threading.Thread(target=_monitor, daemon=True).start()
    print(f"{upper_id} 라인 트레이서를 시작했습니다. (PID: {process.pid})")
    return True


def schedule_next_nav2(robot_id: str) -> None:
    upper_id = robot_id.upper()
    next_step = None
    with control_lock:
        state = robot_states.get(upper_id)
        if state is None:
            return
        if state.get("nav2_active") or state.get("align_pending"):
            return
        queue = state.setdefault("nav2_queue", [])
        if queue:
            next_step = queue.pop(0)
    if next_step:
        command, target, yaw, mode = next_step
        success, reason = dispatch_move_command(upper_id, command=command, target=target, yaw=yaw, mode=mode)
        if not success:
            print(f"{upper_id} 큐 명령 실행 실패: {reason}")


def start_sequence_by_name(robot_id: str, sequence_name: str) -> bool:
    """지정된 이름의 작업 시퀀스를 찾아 로봇의 큐에 추가합니다."""
    upper_id = robot_id.upper()
    if sequence_name not in NAV2_QUEUE_TEMPLATES:
        print(f"ERROR: 시퀀스 템플릿 '{sequence_name}'을(를) 찾을 수 없습니다.")
        return False

    print(f"INFO: {upper_id}에서 '{sequence_name}' 시퀀스를 시작합니다.")
    cancel_nav2_navigation(upper_id, wait=True)
    sequence = []
    for step in NAV2_QUEUE_TEMPLATES[sequence_name]:
        step_spec = COMMAND_SPECS.get(step)
        if not step_spec:
            continue
        sequence.append(
            (
                step,
                step_spec.get("target", (0.0, 0.0)),
                step_spec.get("yaw"),
                step_spec.get("mode", "nav2"),
            )
        )
    enqueue_nav2_sequence(upper_id, sequence)
    return True


def enqueue_nav2_sequence(robot_id: str, sequence: list[tuple[str, tuple[float, float], Optional[float], str]]) -> None:
    upper_id = robot_id.upper()
    with control_lock:
        state = robot_states.setdefault(
            upper_id,
            {
                "sequence_flag": 0,
                "stop_sent": False,
                "pose": (0.0, 0.0),
                "pose_yaw": 0.0,
                "pose_updated_at": 0.0,
                "nav2_label": None,
                "nav2_goal": None,
                "nav2_goal_yaw": 0.0,
                "nav2_paused_label": None,
                "nav2_paused_goal": None,
                "nav2_paused_yaw": None,
                "nav2_manual_active": False,
                "nav2_active": False,
                "align_pending": False,
                "align_target_yaw": 0.0,
                "detection_history": {"ripe": [], "rotten": []},
                "nav2_queue": [],
                "line_tracer_resume": None,
            },
        )
        state.setdefault("sequence_flag", 0)
        state.setdefault("pose", (0.0, 0.0))
        state.setdefault("pose_yaw", 0.0)
        state.setdefault("pose_updated_at", 0.0)
        state["nav2_queue"] = list(sequence)
        state["nav2_paused_label"] = None
        state["nav2_paused_goal"] = None
        state["nav2_paused_yaw"] = None
        state["line_tracer_resume"] = None
    schedule_next_nav2(upper_id)


def start_nav2_navigation(
    robot_id: str,
    target_point: tuple[float, float],
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
                "sequence_flag": 0,
                "stop_sent": False,
                "pose": (0.0, 0.0),
                "pose_yaw": 0.0,
                "pose_updated_at": 0.0,
                "nav2_label": None,
                "nav2_goal": None,
                "nav2_goal_yaw": 0.0,
                "nav2_paused_label": None,
                "nav2_paused_goal": None,
                "nav2_paused_yaw": None,
                "nav2_manual_active": False,
                "nav2_active": False,
                "align_pending": False,
                "align_target_yaw": 0.0,
                "detection_history": {"ripe": [], "rotten": []},
                "nav2_queue": [],
                "line_tracer_resume": None,
            },
        )
        state.setdefault("sequence_flag", 0)
        state.setdefault("pose", (0.0, 0.0))
        state.setdefault("pose_yaw", 0.0)
        state.setdefault("pose_updated_at", 0.0)
        state["nav2_active"] = True
        state["nav2_manual_active"] = False
        state["nav2_label"] = label
        state["nav2_goal"] = target_point
        state["nav2_goal_yaw"] = yaw_value
        state["nav2_paused_label"] = None
        state["nav2_paused_goal"] = None
        state["nav2_paused_yaw"] = None
        state["line_tracer_resume"] = None
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
                target_point[0],
                target_point[1],
                yaw=yaw_value,
            )
            print(f"Nav2 목표 ({target_point[0]:.2f}, {target_point[1]:.2f}) 결과: {result.message}")
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
                if label == "dump@done":
                    with control_lock:
                        state = robot_states.get(upper_id)
                        if state is not None:
                            state["align_pending"] = True
                            state["align_target_yaw"] = 0.0
                    start_alignment_control(upper_id, 0.0)
                else:
                    schedule_next_nav2(upper_id)

    worker = threading.Thread(target=_worker, daemon=True)
    with nav2_motion_lock:
        nav2_motion_thread = worker
    worker.start()
    return True, "NAV2_STARTED"


def send_stop_to_robot(target_id: str, *, kill_line_tracer: bool = True) -> None:
    upper_id = target_id.upper()
    with control_lock:
        target_conn = robot_clients.get(upper_id)
    if not target_conn:
        if kill_line_tracer:
            stop_line_tracer(upper_id)
        return
    try:
        target_conn.sendall(b"STOP\n")
        send_turn_command(upper_id, 0.0, 0.0)
        print(f"{upper_id} 에게 STOP 명령 전송.")
        with control_lock:
            state = robot_states.get(upper_id)
            if state is not None:
                state["stop_sent"] = True
                state["align_pending"] = False
    except OSError as exc:  # noqa: BLE001
        print(f"{upper_id} STOP 명령 전송 실패: {exc}")
    finally:
        if kill_line_tracer:
            stop_line_tracer(upper_id)


def send_turn_command(target_id: str, linear: float, angular: float) -> bool:
    upper_id = target_id.upper()
    with control_lock:
        target_conn = robot_clients.get(upper_id)
    if not target_conn:
        return False
    message = f"TURN:{linear:.3f},{angular:.3f}\n".encode("utf-8")
    try:
        target_conn.sendall(message)
        if abs(angular) > 1e-3 or abs(linear) > 1e-3:
            print(f"{upper_id} TURN 명령 전송: lin={linear:.3f}, ang={angular:.3f}")
        return True
    except OSError as exc:
        print(f"{upper_id} TURN 명령 전송 실패: {exc}")
        return False


ALIGN_THREAD_SLEEP = 0.1
ALIGN_POSE_TIMEOUT = 1.0


def start_alignment_control(robot_id: str, target_yaw: float) -> None:
    upper_id = robot_id.upper()

    def _worker() -> None:
        try:
            while True:
                with control_lock:
                    state = robot_states.get(upper_id)
                    if state is None or not state.get("align_pending"):
                        break
                    pose_yaw = float(state.get("pose_yaw", 0.0))
                    last_update = state.get("pose_updated_at", 0.0)
                if time.monotonic() - last_update > ALIGN_POSE_TIMEOUT:
                    send_turn_command(upper_id, 0.0, 0.0)
                    time.sleep(ALIGN_THREAD_SLEEP)
                    continue
                error = normalize_angle(target_yaw - pose_yaw)
                if abs(error) <= YAW_ALIGNMENT_THRESHOLD:
                    send_turn_command(upper_id, 0.0, 0.0)
                    with control_lock:
                        state = robot_states.get(upper_id)
                        if state is not None:
                            if state.get("nav2_label") == "dump@done":
                                state["sequence_flag"] = 0
                                print(f"INFO: {upper_id}의 dump@done 작업 완료로 sequence_flag가 0으로 초기화됩니다.")
                            state["align_pending"] = False
                            state["nav2_label"] = None
                            state["nav2_goal"] = None
                            state["nav2_goal_yaw"] = 0.0
                            state["stop_sent"] = False
                            history = state.setdefault("detection_history", {"ripe": [], "rotten": []})
                            for buf in history.values():
                                buf.clear()
                    print(f"{upper_id} yaw 정렬 완료")
                    schedule_next_nav2(upper_id)
                    break
                angular = max(-0.6, min(0.6, 1.5 * error))
                if not send_turn_command(upper_id, 0.0, angular):
                    with control_lock:
                        state = robot_states.get(upper_id)
                        if state is not None:
                            state["align_pending"] = False
                    schedule_next_nav2(upper_id)
                    break
                time.sleep(ALIGN_THREAD_SLEEP)
        finally:
            send_turn_command(upper_id, 0.0, 0.0)
            with control_lock:
                alignment_threads.pop(upper_id, None)

    with control_lock:
        if upper_id in alignment_threads:
            return
        thread = threading.Thread(target=_worker, daemon=True)
        alignment_threads[upper_id] = thread
        thread.start()


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
        if state.get("stop_sent"):
            return False
        history = state.setdefault("detection_history", {"ripe": [], "rotten": []})
        buffer = history.setdefault(category, [])
        buffer.append(now)
        while buffer and now - buffer[0] > DETECTION_WINDOW_SECONDS:
            buffer.pop(0)
        if len(buffer) >= DETECTION_COUNT_THRESHOLD:
            buffer.clear()
            spec = COMMAND_SPECS.get(expected_label, {})
            resume_mode = spec.get("mode", "nav2") if isinstance(spec, dict) else "nav2"
            resume_target = spec.get("target", (0.0, 0.0)) if isinstance(spec, dict) else (0.0, 0.0)
            resume_yaw = spec.get("yaw") if isinstance(spec, dict) else None
            state["line_tracer_resume"] = {
                "label": expected_label,
                "target": resume_target,
                "yaw": resume_yaw,
                "mode": resume_mode,
            }
            state["nav2_paused_label"] = expected_label
            state["nav2_paused_goal"] = state.get("nav2_goal")
            state["nav2_paused_yaw"] = state.get("nav2_goal_yaw", 0.0)
            state["nav2_label"] = expected_label
            state["nav2_active"] = False
            state["nav2_manual_active"] = True
            return True
    return False


def resume_nav2_for_robot(robot_id: str) -> bool:
    upper_id = robot_id.upper()
    with control_lock:
        state = robot_states.get(upper_id)
        if state is None:
            return False
        resume_info = state.get("line_tracer_resume")
        label = state.get("nav2_paused_label")
        goal = state.get("nav2_paused_goal")
        yaw = state.get("nav2_paused_yaw")

    if resume_info:
        resume_label = str(resume_info.get("label", "")).lower()
        if not resume_label:
            return False
        resume_mode = str(resume_info.get("mode", COMMAND_SPECS.get(resume_label, {}).get("mode", "nav2")))
        resume_target = resume_info.get("target")
        if isinstance(resume_target, (list, tuple)) and len(resume_target) >= 2:
            resume_target = (float(resume_target[0]), float(resume_target[1]))
        else:
            spec = COMMAND_SPECS.get(resume_label, {})
            raw_target = spec.get("target", (0.0, 0.0)) if isinstance(spec, dict) else (0.0, 0.0)
            if isinstance(raw_target, (list, tuple)) and len(raw_target) >= 2:
                resume_target = (float(raw_target[0]), float(raw_target[1]))
            else:
                resume_target = (0.0, 0.0)
        resume_yaw = resume_info.get("yaw")
        if resume_yaw is None:
            spec = COMMAND_SPECS.get(resume_label, {})
            if isinstance(spec, dict):
                resume_yaw = spec.get("yaw")
        if isinstance(resume_yaw, (int, float)):
            resume_yaw_value: Optional[float] = float(resume_yaw)
        else:
            resume_yaw_value = None
        success, status = dispatch_move_command(
            upper_id,
            command=resume_label,
            target=resume_target,
            yaw=resume_yaw_value,
            mode=resume_mode,
        )
        if success:
            with control_lock:
                state = robot_states.get(upper_id)
                if state is not None:
                    state["line_tracer_resume"] = None
                    state["nav2_paused_label"] = None
                    state["nav2_paused_goal"] = None
                    state["nav2_paused_yaw"] = None
            print(f"{upper_id} 라인 트레이서 재개: {resume_label}")
        else:
            print(f"{upper_id} 라인 트레이서 재개 실패: {status}")
        return success

    if not label or goal is None:
        return False

    spec = COMMAND_SPECS.get(label, {})
    mode = spec.get("mode", "nav2") if isinstance(spec, dict) else "nav2"
    yaw_to_use = yaw if yaw is not None else (spec.get("yaw") if isinstance(spec, dict) else None)

    success, status = dispatch_move_command(
        upper_id,
        command=label,
        target=goal,
        yaw=yaw_to_use if isinstance(yaw_to_use, (int, float)) else None,
        mode=str(mode),
    )
    if success:
        with control_lock:
            state = robot_states.get(upper_id)
            if state is not None:
                state["nav2_paused_label"] = None
                state["nav2_paused_goal"] = None
                state["nav2_paused_yaw"] = None
        print(f"{upper_id} Nav2 재개: {label} → ({goal[0]:.2f}, {goal[1]:.2f})")
    else:
        with control_lock:
            state = robot_states.get(upper_id)
            if state is not None:
                state["nav2_paused_label"] = label
                state["nav2_paused_goal"] = goal
                state["nav2_paused_yaw"] = yaw
        print(f"{upper_id} Nav2 재개 실패: {status}")
    return success


def complete_manual_move(robot_id: str) -> bool:
    """수동 이동(직진 등)이 종료되었을 때 상태를 정리."""
    global nav2_active_robot

    upper_id = robot_id.upper()
    align_required = False
    target_yaw = 0.0
    label: Optional[str] = None
    with control_lock:
        state = robot_states.get(upper_id)
        if state and state.get("nav2_manual_active"):
            state["nav2_manual_active"] = False
            state["nav2_active"] = False
            state["nav2_paused_label"] = None
            state["nav2_paused_goal"] = None
            state["nav2_paused_yaw"] = None
            state["line_tracer_resume"] = None
            history = state.setdefault("detection_history", {"ripe": [], "rotten": []})
            for buf in history.values():
                buf.clear()
            state["stop_sent"] = False
            label = state.get("nav2_label")
            if label == "dump@done":
                align_required = True
                target_yaw = 0.0
                state["align_pending"] = True
                state["align_target_yaw"] = target_yaw
            else:
                state["nav2_label"] = None
                state["nav2_goal"] = None
                state["nav2_goal_yaw"] = 0.0
            if nav2_active_robot == upper_id:
                nav2_active_robot = None
        else:
            return False

    if align_required:
        print(f"{upper_id} 수동 이동 완료: yaw 정렬 시작")
        start_alignment_control(upper_id, target_yaw)
    else:
        if nav2_active_robot == upper_id:
            nav2_active_robot = None
        print(f"{upper_id} 수동 이동 완료: 큐 다음 단계로 진행")
        schedule_next_nav2(upper_id)
    if label in LINE_TRACER_LABELS:
        stop_line_tracer(upper_id)
    return True


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


def trigger_and_handle_auto_stop(client_id: str, conn, y_center_norm: float):
    """
    토마토 감지에 따른 자동 정지 및 로봇팔 제어를 별도 스레드에서 처리합니다.
    handle_client의 메인 루프가 차단되는 것을 방지합니다.
    """
    def _worker():
        upper_id = client_id.upper()
        print(f"INFO: {upper_id}에서 토마토를 감지하여 자동 정지 시퀀스를 시작합니다.")

        with control_lock:
            state = robot_states.get(upper_id)
            if state:
                print(f"INFO: {upper_id} 자동 정지 발생 – 현재 명령을 일시 정지합니다.")

        # 사용자 'stop'과 유사하게 동작하도록 wait=True로 호출 (별도 스레드이므로 안전)
        cancel_nav2_navigation(upper_id, wait=True)
        send_stop_to_robot(client_id)

        if y_center_norm < 1.0 / 3:
            arm_topic = "robot@high"
        elif y_center_norm < 2.0 / 3:
            arm_topic = "robot@middle"
        else:
            arm_topic = "robot@low"
        payload = f"{arm_topic}\n".encode("utf-8")
        try:
            conn.sendall(payload)
            print(f"{client_id} 로봇팔 명령 전송: {arm_topic}")
        except socket.error as exc:
            print(f"{client_id} 로봇팔 명령 전송 실패: {exc}")

    thread = threading.Thread(target=_worker, daemon=True)
    thread.start()


def handle_client(conn, addr, client_id, prefetched_data=b""):
    """클라이언트로부터 영상을 받아 처리하는 함수"""
    global output_frame, nav2_active_robot
    conf_threshold = 0.4
    print(f"클라이언트 {addr}가 연결되었습니다. (ID: {client_id})")
    upper_id = client_id.upper()
    is_robot = client_id.upper().startswith("TURTLE")
    if is_robot:
        with control_lock:
            robot_clients[upper_id] = conn
            robot_states.setdefault(
                upper_id,
                {
                    "sequence_flag": 0,
                    "stop_sent": False,
                    "pose": (0.0, 0.0),
                    "pose_yaw": 0.0,
                    "pose_updated_at": 0.0,
                    "nav2_label": None,
                    "nav2_goal": None,
                    "nav2_goal_yaw": 0.0,
                    "nav2_paused_label": None,
                    "nav2_paused_goal": None,
                    "nav2_paused_yaw": None,
                    "nav2_manual_active": False,
                    "nav2_active": False,
                    "align_pending": False,
                    "align_target_yaw": 0.0,
                    "detection_history": {"ripe": [], "rotten": []},
                    "nav2_queue": [],
                },
            )
            state = robot_states[upper_id]
            state.setdefault("sequence_flag", 0)
            state.setdefault("pose", (0.0, 0.0))
            state.setdefault("pose_yaw", 0.0)
            state.setdefault("pose_updated_at", 0.0)
            state.setdefault("align_pending", False)
            state.setdefault("align_target_yaw", 0.0)
    elif client_id.upper().startswith("CCTV"):
        with control_lock:
            cctv_clients[client_id.upper()] = conn

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
                        parts = [p.strip() for p in pose_payload.split(",")]
                        if len(parts) < 2:
                            raise ValueError
                        x_val = float(parts[0])
                        y_val = float(parts[1])
                        yaw_val = float(parts[2]) if len(parts) >= 3 else None
                    except ValueError:
                        print(f"{client_id} 잘못된 POSE 메시지: {control_line}")
                        continue
                    with control_lock:
                        state = robot_states.setdefault(
                            upper_id,
                            {
                                "sequence_flag": 0,
                                "stop_sent": False,
                                "pose": (x_val, y_val),
                                "pose_yaw": 0.0 if yaw_val is None else yaw_val,
                                "pose_updated_at": time.monotonic(),
                                "nav2_label": None,
                                "nav2_goal": None,
                                "nav2_goal_yaw": 0.0,
                                "nav2_paused_label": None,
                                "nav2_paused_goal": None,
                                "nav2_paused_yaw": None,
                                "nav2_manual_active": False,
                                "nav2_active": False,
                                "align_pending": False,
                                "align_target_yaw": 0.0,
                                "detection_history": {"ripe": [], "rotten": []},
                                "nav2_queue": [],
                                "line_tracer_resume": None,
                            },
                        )
                        state.setdefault("sequence_flag", 0)
                        state.setdefault("pose", (0.0, 0.0))
                        state.setdefault("pose_yaw", 0.0)
                        state.setdefault("pose_updated_at", 0.0)
                        state["pose"] = (x_val, y_val)
                        if yaw_val is not None:
                            state["pose_yaw"] = yaw_val
                        state["pose_updated_at"] = time.monotonic()
                    continue
                if try_handle_sensor_payload(client_id, control_line):
                    continue
                lowered = control_line.lower()
                if lowered == "move@done" and is_robot:
                    complete_manual_move(client_id.upper())
                    continue
                if lowered == "robot@done" and is_robot:
                    upper_id = client_id.upper()
                    with control_lock:
                        state = robot_states.get(upper_id)
                        flag_to_check = state.get("sequence_flag", 0) if state else 0
                        has_resume_hint = bool(
                            state
                            and (
                                state.get("line_tracer_resume")
                                or state.get("nav2_paused_label")
                                or state.get("nav2_paused_goal")
                            )
                        )
                    if has_resume_hint and resume_nav2_for_robot(upper_id):
                        continue
                    if flag_to_check == 1:
                        print(f"INFO: {upper_id}가 플래그 1 상태에서 robot@done을 보내 turtle@go 시퀀스를 시작합니다.")
                        start_sequence_by_name(upper_id, "turtle@go")
                    elif flag_to_check == 2:
                        print(f"INFO: {upper_id}가 플래그 2 상태에서 robot@done을 보내 turtle2@go 시퀀스를 시작합니다.")
                        start_sequence_by_name(upper_id, "turtle2@go")
                    else:
                        if resume_nav2_for_robot(upper_id):
                            continue
                        complete_manual_move(upper_id)
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

            rendered_frame = frame.copy()
            font_base_width = w_orig

            if is_robot:
                # YOLO 추론용 프레임 크기 설정
                yolo_w = 1280
                yolo_h = int(yolo_w * (h_orig / w_orig))  # 종횡비를 유지한 높이 계산
                resized_frame_for_yolo = cv2.resize(frame, (yolo_w, yolo_h))  # YOLO 입력용 프레임
                font_base_width = yolo_w

                # YOLOv8 추론 수행
                with model_lock:
                    results = model(resized_frame_for_yolo, verbose=False)

                # YOLO 입력 크기 대비 원본 크기 스케일 계산
                scale_x = w_orig / yolo_w
                scale_y = h_orig / yolo_h

                # 정규화 좌표를 이용해 바운딩 박스 그리기
                for box in results[0].boxes:
                    conf = float(box.conf[0])
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
                    if class_name in ["ripe", "rotten"]:
                        try:
                            message = f"{class_name}:{x1},{y1}\n"
                            conn.sendall(message.encode("utf-8"))
                        except socket.error as e:
                            print(f"클라이언트로 메시지 전송 실패: {e}")

                    if class_name in {"ripe", "rotten"}:
                        category = "ripe" if class_name == "ripe" else "rotten"
                        if should_trigger_auto_stop(upper_id, category, ((x1 + x2) / 2) / w_orig):
                            y_center_norm = ((y1 + y2) / 2) / h_orig
                            trigger_and_handle_auto_stop(client_id, conn, y_center_norm)

                    cv2.rectangle(rendered_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # 라벨 표시용 글꼴 크기 조정
                    label_font_scale = 0.5 * (w_orig / yolo_w)
                    cv2.putText(
                        rendered_frame,
                        label,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        label_font_scale,
                        (0, 255, 0),
                        2,
                    )

            # FPS 계산
            # fps_frame_count += 1
            # if time.time() - fps_start_time >= 1.0:
            #     display_fps = fps_frame_count / (time.time() - fps_start_time)
            #     fps_frame_count = 0
            #     fps_start_time = time.time()

            # # FPS 표시 (글꼴 크기 포함)
            # fps_font_scale = 0.7 * (w_orig / font_base_width)
            # cv2.putText(rendered_frame, f"FPS: {display_fps:.2f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, fps_font_scale, (0, 0, 255), 2)
            
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
    with control_lock:
        if is_robot:
            stop_line_tracer(client_id)
            robot_clients.pop(client_id.upper(), None)
            robot_states.pop(client_id.upper(), None)
        elif client_id.upper().startswith("CCTV"):
            cctv_clients.pop(client_id.upper(), None)
    conn.close()


def dispatch_move_command(
    target_id: str,
    command: Optional[str] = None,
    origin: tuple[float, float] = (0.0, 0.0),
    target: Optional[tuple[float, float]] = None,
    yaw: Optional[float] = None,
    mode: str = "nav2",
):
    """TURTLEBOT 클라이언트로 이동 명령을 전송하거나 Nav2에 목표를 전달."""
    global nav2_bridge, nav2_active_robot

    upper_id = target_id.upper()
    use_nav2 = mode == "nav2"

    if use_nav2 and nav2_bridge is not None and Nav2Result is not None:
        goal_point = target if target is not None else origin
        goal_yaw = float(yaw) if yaw is not None else 0.0
        if command == "dump@done":
            success, status = start_nav2_navigation(
                upper_id,
                DUMP_STAGE_TARGET,
                "dump@stage",
                DUMP_STAGE_YAW,
            )
            if success:
                with control_lock:
                    state = robot_states.get(upper_id)
                    if state is not None:
                        queue = state.setdefault("nav2_queue", [])
                        queue.insert(0, (DUMP_FINAL_QUEUE_LABEL, goal_point, goal_yaw, "nav2"))
            return success, status
        if command == DUMP_FINAL_QUEUE_LABEL:
            success, status = start_nav2_navigation(upper_id, goal_point, "dump@done", goal_yaw)
            return success, status
        success, status = start_nav2_navigation(upper_id, goal_point, command, yaw)
        return success, status

    with control_lock:
        target_conn = robot_clients.get(upper_id)
        rel_target = (0.0, 0.0)
        if target_conn:
            state = robot_states.setdefault(
                upper_id,
                {
                    "sequence_flag": 0,
                    "stop_sent": False,
                    "pose": (0.0, 0.0),
                    "pose_yaw": 0.0,
                    "pose_updated_at": 0.0,
                    "nav2_label": None,
                    "nav2_goal": None,
                    "nav2_goal_yaw": 0.0,
                    "nav2_paused_label": None,
                    "nav2_paused_goal": None,
                    "nav2_paused_yaw": None,
                    "nav2_manual_active": False,
                    "nav2_active": False,
                    "align_pending": False,
                    "align_target_yaw": 0.0,
                    "detection_history": {"ripe": [], "rotten": []},
                    "nav2_queue": [],
                    "line_tracer_resume": None,
                },
            )
            state.setdefault("sequence_flag", 0)
            state.setdefault("pose", (0.0, 0.0))
            state.setdefault("pose_yaw", 0.0)
            state.setdefault("pose_updated_at", 0.0)

            pose = state.get("pose") or origin
            pose_yaw = float(state.get("pose_yaw", 0.0))

            if command == "ripe@go":
                state["sequence_flag"] = 1
                print(f"INFO: {upper_id} sequence_flag가 ripe@go에 의해 1로 설정되었습니다.")
            elif command == "rotten@go":
                state["sequence_flag"] = 2
                print(f"INFO: {upper_id} sequence_flag가 rotten@go에 의해 2로 설정되었습니다.")

            state["stop_sent"] = False
            state["nav2_label"] = command
            state["nav2_goal_yaw"] = float(yaw) if yaw is not None else 0.0
            state["nav2_paused_label"] = None
            state["nav2_paused_goal"] = None
            state["nav2_paused_yaw"] = None
            state["align_pending"] = False
            state["line_tracer_resume"] = None
            history = state.setdefault("detection_history", {"ripe": [], "rotten": []})
            for buf in history.values():
                buf.clear()

            if mode == "manual_offset":
                rel_target = target if target is not None else (0.0, 0.0)
            elif mode == "manual_absolute":
                tx, ty = target if target is not None else (pose[0], pose[1])
                rel_target = (tx - pose[0], ty - pose[1])
            else:
                rel_target = (0.0, 0.0)

            state["nav2_goal"] = (pose[0] + rel_target[0], pose[1] + rel_target[1])
            state["nav2_active"] = True
            state["nav2_manual_active"] = mode != "nav2"

    if target_conn is None:
        return False, "TARGET_NOT_CONNECTED"

    should_send_move = True

    if command in LINE_TRACER_LABELS and mode != "nav2":
        if not start_line_tracer(upper_id):
            with control_lock:
                state = robot_states.get(upper_id)
                if state is not None:
                    state["nav2_active"] = False
                    state["nav2_manual_active"] = False
            return False, "SCRIPT_FAILED"
        should_send_move = False

    if not should_send_move:
        print(f"{target_id} 라인 트레이서에 제어를 위임합니다. (MOVE 명령 생략)")
        return True, "LINE_TRACER_ACTIVE"

    message = f"MOVE:0.00,0.00->{rel_target[0]:.2f},{rel_target[1]:.2f}\n"
    try:
        target_conn.sendall(message.encode("utf-8"))
        print(f"{target_id} 에게 명령 전송: {message.strip()}")
        if mode != "nav2":
            nav2_active_robot = upper_id
        return True, "OK"
    except OSError as exc:
        print(f"{target_id} 명령 전송 실패: {exc}")
        with control_lock:
            state = robot_states.get(upper_id)
            if state is not None:
                state["nav2_active"] = False
                state["nav2_manual_active"] = False
        if command in LINE_TRACER_LABELS:
            stop_line_tracer(upper_id)
        return False, "SEND_FAILED"


def handle_user_client(conn, addr, client_id, prefetched_data=b""):
    """USERxx 명령을 받아 대응되는 TURTLEBOTxx로 전달."""
    print(f"사용자 채널 {addr} 접속 (ID: {client_id})")
    suffix = client_id.upper()[4:] if len(client_id) >= 4 else ""
    target_id = f"TURTLE{suffix}" if suffix else None
    command_specs = COMMAND_SPECS
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
            if message.startswith("[") and "]" in message:
                bracket_end = message.find("]")
                target_label = message[1:bracket_end].strip()
                #payload = message[bracket_end + 1 :].lstrip()
                payload = message
                target_key = target_label.upper()
                if target_key.startswith("CCTV"):
                    with control_lock:
                        target_conn = cctv_clients.get(target_key)
                    if not target_conn:
                        conn.sendall(b"ERROR:TARGET_NOT_FOUND\n")
                        continue
                    try:
                        target_conn.sendall((payload.rstrip("\r\n") + "\n").encode("utf-8"))
                        conn.sendall(b"CMD_OK\n")
                    except OSError as exc:
                        print(f"{client_id} -> {target_key} 메시지 전달 실패: {exc}")
                        conn.sendall(b"ERROR:FORWARD_FAILED\n")
                    continue

            command = message.lower()
            spec = command_specs.get(command)
            if spec:
                if not target_id:
                    conn.sendall(b"ERROR:TARGET_UNKNOWN\n")
                    continue
                cancel_nav2_navigation(target_id, wait=True)
                mode = spec.get("mode", "nav2")
                target = spec.get("target", (0.0, 0.0))
                yaw_value = spec.get("yaw")
                success, reason = dispatch_move_command(
                    target_id,
                    command=command,
                    target=target,
                    yaw=yaw_value,
                    mode=mode,
                )
                response = b"CMD_OK\n" if success else f"ERROR:{reason}\n".encode("utf-8")
                conn.sendall(response)
            elif command in NAV2_QUEUE_TEMPLATES:
                if not target_id:
                    conn.sendall(b"ERROR:TARGET_UNKNOWN\n")
                    continue
                cancel_nav2_navigation(target_id, wait=True)
                sequence = []
                for step in NAV2_QUEUE_TEMPLATES[command]:
                    step_spec = command_specs.get(step)
                    if not step_spec:
                        continue
                    sequence.append(
                        (
                            step,
                            step_spec.get("target", (0.0, 0.0)),
                            step_spec.get("yaw"),
                            step_spec.get("mode", "nav2"),
                        )
                    )
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


def pose_debug_worker() -> None:
    """주기적으로 각 로봇의 최신 오도메트리를 출력."""
    while True:
        time.sleep(POSE_LOG_INTERVAL_SECONDS)
        with control_lock:
            snapshot = {
                robot_id: (
                    state.get("pose"),
                    state.get("pose_yaw"),
                    state.get("pose_updated_at", 0.0),
                )
                for robot_id, state in robot_states.items()
            }
        if not snapshot:
            continue
        now = time.monotonic()
        for robot_id, (pose, yaw, updated_at) in snapshot.items():
            if pose is None:
                continue
            if updated_at:
                age = now - updated_at
                print(
                    f"[POSE_DEBUG] {robot_id}: x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={yaw:.3f} "
                    f"(업데이트 {age:.1f}초 전)"
                )
            else:
                print(
                    f"[POSE_DEBUG] {robot_id}: x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={yaw:.3f} "
                    "(업데이트 기록 없음)"
                )


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

    pose_thread = threading.Thread(target=pose_debug_worker, daemon=True)
    pose_thread.start()

    # Flask 앱 실행
    app.run(host='0.0.0.0', port=5000, debug=False)

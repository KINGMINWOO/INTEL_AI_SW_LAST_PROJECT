"""TurtleBot 전용 스트리밍 클라이언트 (ROS2 오도메트리 전송 포함)."""

import math
import socket
import struct
import sys
import threading
import time
from typing import Optional, Tuple

import cv2

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
except ImportError:  # noqa: F401
    rclpy = None  # type: ignore
    Odometry = None  # type: ignore
    Node = None  # type: ignore
    Twist = None  # type: ignore

AUTH_PROMPT = "AUTH_REQUEST"
AUTH_OK = "AUTH_OK"
CLIENT_ID = "TURTLE01"
CLIENT_PASSWORD = "TURTLE1234"

#SERVER_ADDR: Tuple[str, int] = ("10.10.16.29", 9999)
SERVER_ADDR: Tuple[str, int] = ("127.0.0.1", 9999)
FRAME_SIZE = (640, 480)  # (width, height)
TARGET_FPS = 10          # 0 => no sleep, otherwise throttle
JPEG_QUALITY = 85
ODOM_TOPIC = "/odom"
POSE_SEND_INTERVAL = 0.2  # seconds
CAMERA_FAILURE_LIMIT = 5
CAMERA_FORMAT_CANDIDATES = ["MJPG", "YUYV", "YUY2", "UYVY", "NV12", "YU12"]
YAW_TOLERANCE = 0.02
ANGULAR_KP = 1.5
MAX_ANGULAR_SPEED = 0.9


def quaternion_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """geometry_msgs/Quaternion → yaw(라디안) 변환."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def decode_fourcc(value: float) -> str:
    """OpenCV 반환 값을 FOURCC 문자열로 변환."""
    code = int(value)
    chars = [chr((code >> (8 * i)) & 0xFF) for i in range(4)]
    result = "".join(chars)
    return result if result.strip("\x00") else "----"


def try_set_fourcc(cap: cv2.VideoCapture, fourcc: str) -> bool:
    """지정한 FOURCC를 설정하고 실제 적용 여부를 확인."""
    code = cv2.VideoWriter_fourcc(*fourcc)
    if not cap.set(cv2.CAP_PROP_FOURCC, code):
        return False
    applied = decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))
    return applied == fourcc


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class OdometryBridge:
    """ROS2 오도메트리를 구독하고 cmd_vel을 발행하는 간단한 브리지."""

    def __init__(self, odom_topic: str = ODOM_TOPIC, cmd_vel_topic: str = "/cmd_vel") -> None:
        self.odom_topic = odom_topic
        self.cmd_vel_topic = cmd_vel_topic
        self._lock = threading.Lock()
        self._latest: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._has_pose = False
        self._origin: Optional[Tuple[float, float]] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._node: Optional["OdometryBridge._OdomNode"] = None
        self._owns_context = False

    class _OdomNode(Node):
        def __init__(self, parent: "OdometryBridge", odom_topic: str, cmd_vel_topic: str) -> None:
            super().__init__("turtle_client_odom_bridge")
            self._parent = parent
            self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
            self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)

        def _odom_callback(self, msg: Odometry) -> None:
            pose = msg.pose.pose
            self._parent._set_pose(
                float(pose.position.x),
                float(pose.position.y),
                quaternion_to_yaw(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
            )

        def publish_velocity(self, linear_x: float, angular_z: float) -> None:
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self._cmd_pub.publish(twist)

    def start(self) -> bool:
        if rclpy is None or Node is None or Odometry is None or Twist is None:
            print("rclpy 패키지를 찾을 수 없어 오도메트리 구독을 건너뜁니다.")
            return False
        if self._running:
            return True
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_context = True
            else:
                self._owns_context = False
            self._node = self._OdomNode(self, self.odom_topic, self.cmd_vel_topic)
            self._running = True
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
            print(f"ROS2 오도메트리 구독을 시작합니다: {self.odom_topic}")
            return True
        except Exception as exc:  # noqa: BLE001
            print(f"오도메트리 브리지 초기화 실패: {exc}")
            self.stop()
            return False

    def _spin(self) -> None:
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
        finally:
            if self._node is not None:
                try:
                    self._node.destroy_node()
                except Exception:  # noqa: BLE001
                    pass
                self._node = None
            if self._owns_context and rclpy is not None and rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:  # noqa: BLE001
                    pass
            self._owns_context = False
            self._running = False

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

    def _set_pose(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            if self._origin is None:
                self._origin = (x, y)
            self._latest = (x, y, yaw)
            self._has_pose = True

    def get_pose(self, *, relative: bool = False) -> Tuple[float, float, float]:
        with self._lock:
            x, y, yaw = self._latest
            if relative and self._origin is not None:
                ox, oy = self._origin
                return (x - ox, y - oy, yaw)
            return (x, y, yaw)

    def has_pose(self) -> bool:
        with self._lock:
            return self._has_pose

    def send_velocity(self, linear_x: float, angular_z: float) -> None:
        if self._node is None:
            return
        self._node.publish_velocity(linear_x, angular_z)

    def to_world(self, point: Tuple[float, float]) -> Tuple[float, float]:
        with self._lock:
            if self._origin is None:
                return point
            ox, oy = self._origin
        return (point[0] + ox, point[1] + oy)

    def to_relative(self, x: float, y: float) -> Tuple[float, float]:
        with self._lock:
            if self._origin is None:
                return (x, y)
            ox, oy = self._origin
        return (x - ox, y - oy)


class MotionController:
    """간단한 직선/회전 제어를 수행하는 컨트롤러."""

    def __init__(self, bridge: OdometryBridge, notify_cb) -> None:
        self._bridge = bridge
        self._notify_cb = notify_cb
        self._target: Optional[Tuple[float, float]] = None
        self._manual_active = False
        self._manual_linear = 0.0
        self._manual_angular = 0.0
        self._lock = threading.Lock()
        self._cancel_requested = False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def move_to(self, target: Tuple[float, float]) -> None:
        if not self._bridge.start():
            print("ROS2 브리지를 시작할 수 없어 이동 명령을 건너뜁니다.")
            return
        with self._lock:
            self._target = target
            self._manual_active = False
            self._cancel_requested = False
        print(f"[모션] 목표 좌표 rel({target[0]:.2f}, {target[1]:.2f}) 이동 시작")

    def set_manual_velocity(self, linear: float, angular: float) -> None:
        if not self._bridge.start():
            print("ROS2 브리지를 시작할 수 없어 수동 제어를 건너뜁니다.")
            return
        with self._lock:
            self._target = None
            self._manual_linear = linear
            self._manual_angular = angular
            self._manual_active = abs(linear) > 1e-4 or abs(angular) > 1e-4
            self._cancel_requested = False
        if not self._manual_active:
            self._bridge.send_velocity(0.0, 0.0)

    def cancel(self) -> None:
        with self._lock:
            self._target = None
            self._manual_active = False
            self._cancel_requested = True
        self._bridge.send_velocity(0.0, 0.0)
        print("[모션] 이동 취소 및 정지")

    def stop(self) -> None:
        self._running = False
        self.cancel()
        self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        kp = 0.8
        max_speed = 0.35
        min_speed = 0.05
        goal_tolerance = 0.03
        lateral_tolerance = 0.1

        while self._running:
            with self._lock:
                target = self._target
                manual_active = self._manual_active
                manual_linear = self._manual_linear
                manual_angular = self._manual_angular
                cancelled = self._cancel_requested

            if cancelled or (target is None and not manual_active):
                time.sleep(0.05)
                continue

            if not self._bridge.has_pose():
                self._bridge.send_velocity(0.0, 0.0)
                time.sleep(0.05)
                continue

            rel_x, rel_y, yaw = self._bridge.get_pose(relative=True)

            if manual_active:
                self._bridge.send_velocity(manual_linear, manual_angular)
                time.sleep(0.05)
                continue

            dx = target[0] - rel_x
            dy = target[1] - rel_y
            distance = math.hypot(dx, dy)
            forward = dx * math.cos(yaw) + dy * math.sin(yaw)
            lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)

            if distance <= goal_tolerance:
                self._bridge.send_velocity(0.0, 0.0)
                with self._lock:
                    self._target = None
                    self._cancel_requested = False
                self._notify_cb("move@done")
                time.sleep(0.05)
                continue

            if abs(lateral) > lateral_tolerance:
                print(f"[모션] 좌우 오차 {lateral:.3f}m (yaw={yaw:.3f})")

            speed = kp * forward
            speed = max(min(speed, max_speed), min_speed)
            self._bridge.send_velocity(speed, 0.0)
            time.sleep(0.05)
def configure_capture() -> Tuple[cv2.VideoCapture, float]:
    """카메라를 빠르게 초기화하고 센서 FPS를 반환."""
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다.")

    chosen_fourcc = None
    for candidate in CAMERA_FORMAT_CANDIDATES:
        if try_set_fourcc(cap, candidate):
            chosen_fourcc = candidate
            break
    if chosen_fourcc is None:
        print("지원되는 FOURCC 포맷을 적용하지 못했습니다. 기본 설정으로 시도합니다.")
    else:
        print(f"카메라 FOURCC 적용: {chosen_fourcc}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])
    applied_fourcc = decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))

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
            f" (FOURCC {applied_fourcc})"
            f" (요청 {requested_fps:.1f}fps, 전송은 {requested_fps:.1f}fps로 제한)")
    else:
        print(f"카메라 설정: {actual_w:.0f}x{actual_h:.0f} @ {sensor_fps:.1f}fps (FOURCC {applied_fourcc})")

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


def handle_server_message(line: str, motion: Optional[MotionController]) -> None:
    if not line:
        return
    upper = line.strip().upper()
    if line.startswith("MOVE:"):
        print(f"MOVE 명령 수신: {line}")
        if motion is None:
            return
        try:
            _, payload = line.split(":", 1)
            start_raw, target_raw = payload.split("->", 1)
            tx_str, ty_str = target_raw.split(",", 1)
            tx = float(tx_str)
            ty = float(ty_str)
            motion.move_to((tx, ty))
        except ValueError:
            print(f"MOVE 명령 파싱 실패: {line}")
    elif upper == "STOP":
        print("STOP 명령 수신: 즉시 정지합니다.")
        if motion is not None:
            motion.cancel()
    elif line.startswith("TURN:"):
        if motion is None:
            return
        try:
            _, payload = line.split(":", 1)
            parts = [p.strip() for p in payload.split(",") if p.strip()]
            if not parts:
                return
            if len(parts) == 1:
                linear = 0.0
                angular = float(parts[0])
            else:
                linear = float(parts[0])
                angular = float(parts[1])
        except ValueError:
            print(f"TURN 명령 파싱 실패: {line}")
            return
        motion.set_manual_velocity(linear, angular)
    else:
        print(f"서버 메시지: {line}")


def receive_async(sock: socket.socket, motion: Optional[MotionController]) -> None:
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
                    handle_server_message(line, motion)
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


def send_pose_message(sock: socket.socket, pose: Tuple[float, float, float]) -> None:
    header = struct.pack(">L", 0)
    x, y, yaw = pose
    payload = f"POSE:{x:.3f},{y:.3f},{yaw:.3f}\n".encode("utf-8")
    try:
        sock.sendall(header + payload)
    except OSError as exc:
        print(f"POSE 메시지 전송 실패: {exc}")


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

    odom_bridge = OdometryBridge()
    if not odom_bridge.start():
        print("오도메트리 브리지를 초기화하지 못해 클라이언트를 종료합니다.")
        sock.close()
        sys.exit(1)
    motion = MotionController(odom_bridge, lambda msg: send_control_message(sock, msg))

    threading.Thread(target=receive_async, args=(sock, motion), daemon=True).start()
    threading.Thread(target=command_prompt, args=(stop_event, sock), daemon=True).start()

    cap: Optional[cv2.VideoCapture] = None

    try:
        cap, sensor_fps = configure_capture()
    except RuntimeError as exc:
        print(exc)
        motion.stop()
        odom_bridge.stop()
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
        next_pose_send = time.perf_counter()
        camera_failures = 0
        while True:
            cap.grab()
            ret, frame = cap.read()
            if not ret:
                camera_failures += 1
                if camera_failures >= CAMERA_FAILURE_LIMIT:
                    print("프레임을 반복적으로 읽지 못했습니다. 종료합니다.")
                    break
                print("프레임을 읽지 못했습니다. 잠시 후 재시도합니다.")
                time.sleep(0.2)
                continue

            camera_failures = 0

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

            now = time.perf_counter()
            if now >= next_pose_send:
                if odom_bridge.has_pose():
                    pose_world = odom_bridge.get_pose(relative=False)
                    rel_x, rel_y = odom_bridge.to_relative(pose_world[0], pose_world[1])
                    pose = (rel_x, rel_y, pose_world[2])
                    send_pose_message(sock, pose)
                next_pose_send = now + POSE_SEND_INTERVAL

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
        if cap is not None:
            cap.release()
        motion.stop()
        odom_bridge.stop()
        sock.close()


if __name__ == "__main__":
    main()

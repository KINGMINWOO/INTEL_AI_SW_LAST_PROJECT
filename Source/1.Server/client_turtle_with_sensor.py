"TurtleBot 전용 스트리밍 클라이언트 (ROS2, 아두이노 연동)."

import math
import socket
import struct
import sys
import threading
import time
from typing import Optional, Tuple, Callable

import cv2

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.time import Time
except ImportError:
    rclpy = None
    Odometry = None
    Node = None
    Twist = None
    Time = None

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    GPIO = None

try:
    import serial
except ImportError:
    serial = None

# --- 설정 상수 ---
AUTH_PROMPT = "AUTH_REQUEST"
AUTH_OK = "AUTH_OK"
CLIENT_ID = "TURTLE01"
CLIENT_PASSWORD = "TURTLE1234"
SERVER_ADDR: Tuple[str, int] = ("192.168.0.4", 9999)
FRAME_SIZE = (1280, 720)
TARGET_FPS = 40
JPEG_QUALITY = 85
ODOM_TOPIC = "/odom"
POSE_SEND_INTERVAL = 0.2
CAMERA_FAILURE_LIMIT = 5
CAMERA_FORMAT_CANDIDATES = ["MJPG", "YUYV", "YUY2", "UYVY", "NV12", "YU12"]
YAW_TOLERANCE = 0.02
ANGULAR_KP = 1.5
MAX_ANGULAR_SPEED = 0.9
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 9600


def quaternion_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """ROS 쿼터니언을 평면 yaw 각도로 변환한다."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def decode_fourcc(value: float) -> str:
    """OpenCV FOURCC 값을 사람이 읽을 수 있는 문자열로 바꾼다."""
    code = int(value)
    chars = [chr((code >> (8 * i)) & 0xFF) for i in range(4)]
    result = "".join(chars)
    return result if result.strip("\x00") else "----"

def try_set_fourcc(cap: cv2.VideoCapture, fourcc: str) -> bool:
    """지원 포맷 후보를 시도하면서 카메라 FOURCC를 세팅한다."""
    code = cv2.VideoWriter_fourcc(*fourcc)
    if not cap.set(cv2.CAP_PROP_FOURCC, code):
        return False
    applied = decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))
    return applied == fourcc

def normalize_angle(angle: float) -> float:
    """각도를 -pi~pi 범위로 정규화한다."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

class ArduinoBridge:
    """아두이노와 USB 시리얼 통신을 담당하는 브리지."""
    def __init__(self, port: str, baudrate: int, message_callback: Callable[[str], None]):
        if serial is None:
            raise ImportError("pyserial 라이브러리가 설치되지 않았습니다.")
        self._port = port
        self._baudrate = baudrate
        self._message_callback = message_callback
        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        try:
            self._serial = serial.Serial(self._port, self._baudrate, timeout=1)
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            print(f"🔌 Arduino 브리지 시작: {self._port} @ {self._baudrate}bps")
            return True
        except serial.SerialException as e:
            print(f"Arduino 브리지 시작 실패: {e}")
            self._serial = None
            return False

    def _read_loop(self) -> None:
        while self._running and self._serial:
            try:
                if self._serial.in_waiting > 0:
                    line = self._serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"[Arduino → Pi]: {line}")
                        self._message_callback(line)
            except (OSError, serial.SerialException):
                print("Arduino 연결이 끊어졌습니다.")
                break
            time.sleep(0.05)

    def write(self, message: str) -> bool:
        if self._serial and self._serial.is_open:
            try:
                print(f"[Pi → Arduino]: {message}")
                self._serial.write((message + '\n').encode('utf-8'))
                return True
            except (OSError, serial.SerialException) as e:
                print(f"Arduino에 메시지 전송 실패: {e}")
        return False

    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
            print("Arduino 브리지 종료.")

class OdometryBridge:
    """ROS2 오도메트리 구독, 라인 센서 읽기, cmd_vel 발행을 통합한 브리지."""
    def __init__(self, odom_topic: str = ODOM_TOPIC, cmd_vel_topic: str = "/cmd_vel") -> None:
        self.odom_topic = odom_topic
        self.cmd_vel_topic = cmd_vel_topic
        self._lock = threading.Lock()
        self._latest_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._latest_sensor: Tuple[int, int] = (0, 0)
        self._has_pose = False
        self._origin: Optional[Tuple[float, float]] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._node: Optional["OdometryBridge._BridgeNode"] = None
        self._owns_context = False

    class _BridgeNode(Node):
        def __init__(self, parent: "OdometryBridge", odom_topic: str, cmd_vel_topic: str) -> None:
            super().__init__("turtle_client_bridge")
            self._parent = parent
            self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
            self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
            self._gpio_initialized = False
            if GPIO:
                try:
                    GPIO.setmode(GPIO.BCM)
                    self._left_pin = 17
                    self._right_pin = 27
                    GPIO.setup(self._left_pin, GPIO.IN)
                    GPIO.setup(self._right_pin, GPIO.IN)
                    self.create_timer(0.05, self._read_line_sensor)
                    self.get_logger().info("📡 Line Sensor GPIO integration enabled.")
                    self._gpio_initialized = True
                except Exception as e:
                    self.get_logger().error(f"Failed to initialize GPIO for line sensor: {e}")
            else:
                self.get_logger().warn("RPi.GPIO not found, line sensor is disabled.")

        def _odom_callback(self, msg: Odometry) -> None:
            pose = msg.pose.pose
            self._parent._set_pose(float(pose.position.x), float(pose.position.y), quaternion_to_yaw(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))

        def _read_line_sensor(self) -> None:
            if not self._gpio_initialized:
                return
            try:
                self._parent._set_line_sensor((GPIO.input(self._left_pin), GPIO.input(self._right_pin)))
            except Exception as e:
                self.get_logger().error(f"Error reading line sensor data: {e}")

        def publish_velocity(self, linear_x: float, angular_z: float) -> None:
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self._cmd_pub.publish(twist)

        def cleanup_gpio(self) -> None:
            if self._gpio_initialized:
                GPIO.cleanup()
                self.get_logger().info("GPIO cleaned up.")
        
        def get_ros_time(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

    def start(self) -> bool:
        """rclpy 노드를 초기화하고 오도메트리/센서 구독을 시작한다."""
        if rclpy is None:
            print("rclpy 패키지를 찾을 수 없어 ROS2 기능을 건너뜁니다.")
            return False
        if self._running:
            return True
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_context = True
            else:
                self._owns_context = False
            self._node = self._BridgeNode(self, self.odom_topic, self.cmd_vel_topic)
            self._running = True
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
            print(f"ROS2 브리지 시작: 오도메트리({self.odom_topic}), 라인센서(GPIO)")
            return True
        except Exception as exc:
            print(f"ROS2 브리지 초기화 실패: {exc}")
            self.stop()
            return False

    def _spin(self) -> None:
        """별도 스레드에서 rclpy 이벤트 루프를 돌린다."""
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
        finally:
            if self._node is not None:
                try:
                    if hasattr(self._node, "cleanup_gpio"): self._node.cleanup_gpio()
                    self._node.destroy_node()
                except Exception: pass
                self._node = None
            if self._owns_context and rclpy and rclpy.ok():
                try: rclpy.shutdown()
                except Exception: pass
            self._owns_context = False
            self._running = False

    def stop(self) -> None:
        """ROS2 브리지 스레드와 노드를 안전하게 종료한다."""
        if not self._running: return
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

    def _set_pose(self, x: float, y: float, yaw: float) -> None:
        """오도메트리 최신값을 저장한다."""
        with self._lock:
            if self._origin is None: self._origin = (x, y)
            self._latest_pose = (x, y, yaw)
            self._has_pose = True
    
    def _set_line_sensor(self, sensor_data: Tuple[int, int]) -> None:
        """라인 센서 값을 갱신한다."""
        with self._lock:
            self._latest_sensor = sensor_data

    def get_pose(self, *, relative: bool = False) -> Tuple[float, float, float]:
        with self._lock:
            x, y, yaw = self._latest_pose
            if relative and self._origin is not None:
                ox, oy = self._origin
                return (x - ox, y - oy, yaw)
            return (x, y, yaw)

    def get_line_sensor(self) -> Tuple[int, int]:
        with self._lock:
            return self._latest_sensor

    def has_pose(self) -> bool:
        with self._lock:
            return self._has_pose

    def send_velocity(self, linear_x: float, angular_z: float) -> None:
        if self._node: self._node.publish_velocity(linear_x, angular_z)

    def get_time(self) -> float:
        if self._node: return self._node.get_ros_time()
        return time.time()

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
    """직선/회전/라인추적 제어를 수행하는 통합 컨트롤러."""
    def __init__(self, bridge: OdometryBridge, notify_cb: Callable[[str], None]) -> None:
        self._bridge = bridge
        self._notify_cb = notify_cb
        self._lock = threading.Lock()
        self._running = True
        self._target: Optional[Tuple[float, float]] = None
        self._manual_active = False
        self._manual_linear = 0.0
        self._manual_angular = 0.0
        self._line_tracing_active = False
        self._cancel_requested = False
        self._lt_left_sweep_angle = math.radians(90)
        self._lt_right_sweep_angle = math.radians(90)
        self._lt_scan_angular_speed = 0.6
        self._lt_linear_speed = 0.02
        self._lt_angular_speed = 0.1
        self._lt_calibration_active = False
        self._lt_calibration_direction = 1.0
        self._lt_calibration_remaining = 0.0
        self._lt_last_calibration_time = 0.0
        self._lt_tracking_started = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def move_to(self, target: Tuple[float, float]) -> None:
        """상대 좌표 목표까지 이동하는 Nav2 대체 로직."""
        if not self._bridge.start(): return
        with self._lock:
            self._target = target
            self._manual_active = False
            self._line_tracing_active = False
            self._cancel_requested = False
        print(f"[모션] 목표 좌표 rel({target[0]:.2f}, {target[1]:.2f}) 이동 시작")

    def set_manual_velocity(self, linear: float, angular: float) -> None:
        """사용자 지정 선속도/각속도로 직접 제어한다."""
        if not self._bridge.start(): return
        with self._lock:
            self._target = None
            self._line_tracing_active = False
            self._manual_linear = linear
            self._manual_angular = angular
            self._manual_active = abs(linear) > 1e-4 or abs(angular) > 1e-4
            self._cancel_requested = False
        if not self._manual_active: self._bridge.send_velocity(0.0, 0.0)

    def start_line_tracing(self) -> None:
        """라인 센서 기반 자동 주행을 시작한다."""
        if not self._bridge.start(): return
        print("[모션] 라인 추적 시작")
        with self._lock:
            self._target = None
            self._manual_active = False
            self._cancel_requested = False
            self._line_tracing_active = True
            self._lt_calibration_active = True
            self._lt_tracking_started = False
            self._lt_calibration_direction = 1.0
            self._lt_calibration_remaining = self._lt_left_sweep_angle
            self._lt_last_calibration_time = self._bridge.get_time()
        print("[모션] 라인 추적 준비: 센서 [1,1]을 찾기 위해 좌/우 회전을 시작합니다.")

    def cancel(self) -> None:
        """현재 진행 중인 모든 동작을 중단하고 서버에 알린다."""
        with self._lock:
            if self._target is None and not self._manual_active and not self._line_tracing_active: return
            self._target = None
            self._manual_active = False
            self._line_tracing_active = False
            self._cancel_requested = True
        self._bridge.send_velocity(0.0, 0.0)
        self._notify_cb("stop@done")
        print("[모션] 모든 이동 취소 및 정지")

    def stop(self) -> None:
        """컨트롤러 스레드를 종료한다."""
        self._running = False
        self.cancel()
        self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        """제어 스레드 메인 루프."""
        while self._running:
            with self._lock:
                is_line_tracing = self._line_tracing_active
                is_manual = self._manual_active
                target_point = self._target
                manual_linear = self._manual_linear
                manual_angular = self._manual_angular
                is_cancelled = self._cancel_requested
            if is_cancelled or (not is_line_tracing and not is_manual and target_point is None):
                time.sleep(0.05)
                continue
            if not self._bridge.has_pose():
                self._bridge.send_velocity(0.0, 0.0)
                time.sleep(0.05)
                continue
            if is_line_tracing: self._line_tracer_step()
            elif is_manual: self._bridge.send_velocity(manual_linear, manual_angular)
            elif target_point is not None: self._move_to_step(target_point)
            time.sleep(0.05)

    def _move_to_step(self, target: Tuple[float, float]) -> None:
        """목표점을 향해 P 제어로 이동한다."""
        kp = 0.8
        max_speed, min_speed, goal_tolerance = 0.35, 0.05, 0.03
        rel_x, rel_y, yaw = self._bridge.get_pose(relative=True)
        dx, dy = target[0] - rel_x, target[1] - rel_y
        distance = math.hypot(dx, dy)
        if distance <= goal_tolerance:
            self._bridge.send_velocity(0.0, 0.0)
            with self._lock: self._target = None; self._cancel_requested = False
            self._notify_cb("move@done")
            return
        target_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(target_yaw - yaw)
        linear_speed, angular_speed = 0.0, 0.0
        if abs(yaw_error) > YAW_TOLERANCE:
            angular_speed = ANGULAR_KP * yaw_error
            angular_speed = max(min(angular_speed, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)
        else:
            linear_speed = kp * distance
            linear_speed = max(min(linear_speed, max_speed), min_speed)
        self._bridge.send_velocity(linear_speed, angular_speed)

    def _line_tracer_step(self) -> None:
        """라인 센서 패턴에 따라 속도를 조절한다."""
        raw_left, raw_right = self._bridge.get_line_sensor()
        with self._lock:
            if not self._line_tracing_active: return
            if self._lt_calibration_active:
                if raw_left == 1 and raw_right == 1:
                    print("[모션] 센서 [1,1] 감지 → 라인 추적 준비 완료.")
                    self._lt_calibration_active = False
                    self._lt_tracking_started = False
                    self._bridge.send_velocity(0.0, 0.0)
                    print("[모션] 라인 추적 준비 단계가 종료되었습니다. 라인 트레이싱을 시작합니다.")
                    return
                now = self._bridge.get_time()
                delta = now - self._lt_last_calibration_time
                self._lt_last_calibration_time = now
                self._lt_calibration_remaining -= abs(self._lt_scan_angular_speed) * delta
                angular_z = self._lt_calibration_direction * self._lt_scan_angular_speed
                self._bridge.send_velocity(0.0, angular_z)
                if self._lt_calibration_remaining <= 0.0:
                    if self._lt_calibration_direction > 0:
                        self._lt_calibration_direction = -1.0
                        self._lt_calibration_remaining = self._lt_right_sweep_angle
                        print("[모션] 라인 추적 준비: 오른쪽으로 회전")
                    else:
                        self._lt_calibration_direction = 1.0
                        self._lt_calibration_remaining = self._lt_left_sweep_angle
                        print("[모션] 라인 추적 준비: 왼쪽으로 회전")
                return
            left_on_line, right_on_line = raw_left == 1, raw_right == 1
            linear_x, angular_z = 0.0, 0.0
            if left_on_line and right_on_line:
                linear_x = self._lt_linear_speed
                if not self._lt_tracking_started:
                    print("[모션] 라인 추적 시작됨.")
                    self._lt_tracking_started = True
            elif left_on_line and not right_on_line: angular_z = self._lt_angular_speed
            elif right_on_line and not left_on_line: angular_z = -self._lt_angular_speed
            else:
                if self._lt_tracking_started:
                    self._bridge.send_velocity(0.0, 0.0)
                    print("[모션] 센서 [0,0] 감지 → 라인 추적 완료.")
                    self._line_tracing_active = False
                    self._lt_tracking_started = False
                    self._notify_cb("trace@done")
                    return
                else:
                    angular_z = self._lt_scan_angular_speed * self._lt_calibration_direction
            self._bridge.send_velocity(linear_x, angular_z)

def configure_capture() -> Optional[Tuple[cv2.VideoCapture, float]]:
    """카메라 장치를 열고 해상도·FPS·FOURCC를 맞춘다."""
    try:
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened(): raise RuntimeError("카메라를 열 수 없습니다.")
        chosen_fourcc = None
        for candidate in CAMERA_FORMAT_CANDIDATES:
            if try_set_fourcc(cap, candidate):
                chosen_fourcc = candidate
                break
        if chosen_fourcc: print(f"카메라 FOURCC 적용: {chosen_fourcc}")
        else: print("지원되는 FOURCC 포맷을 적용하지 못했습니다.")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])
        cap.set(cv2.CAP_PROP_FPS, TARGET_FPS if TARGET_FPS > 0 else 30.0)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        w, h, fps = cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT), cap.get(cv2.CAP_PROP_FPS)
        print(f"카메라 설정: {w:.0f}x{h:.0f} @ {fps:.1f}fps (FOURCC {decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))})")
        return cap, fps
    except Exception as e:
        print(f"카메라 설정 실패: {e}")
        return None

def _recv_line(sock: socket.socket) -> Optional[str]:
    """서버로부터 개행 단위 문자열을 수신한다."""
    data = bytearray()
    while b"\n" not in data:
        chunk = sock.recv(1024)
        if not chunk: return None
        data.extend(chunk)
    try:
        return data.split(b"\n", 1)[0].decode("utf-8").strip()
    except UnicodeDecodeError: return None

def authenticate(sock: socket.socket) -> bool:
    """서버와 인증 핸드셰이크를 수행한다."""
    if _recv_line(sock) != AUTH_PROMPT: return False
    sock.sendall(f"{CLIENT_ID}:{CLIENT_PASSWORD}\n".encode("utf-8"))
    return _recv_line(sock) == AUTH_OK

def handle_server_message(line: str, motion: MotionController, arduino: ArduinoBridge) -> None:
    """서버에서 오는 제어 명령을 해석해 모션/아두이노 브리지에 전달한다."""
    if not line: return
    if line.startswith(("robot@", "dump@start")):
        arduino.write(line)
        return
    upper = line.strip().upper()
    if upper == "TRACE_LINE": motion.start_line_tracing()
    elif line.startswith("MOVE:"):
        try:
            _, payload = line.split(":", 1)
            _, target_raw = payload.split("->", 1)
            tx, ty = [float(v) for v in target_raw.split(",", 1)]
            motion.move_to((tx, ty))
        except ValueError: print(f"MOVE 명령 파싱 실패: {line}")
    elif upper == "STOP": motion.cancel()
    elif line.startswith("TURN:"):
        try:
            _, payload = line.split(":", 1)
            parts = [p.strip() for p in payload.split(",") if p.strip()]
            if len(parts) == 1: linear, angular = 0.0, float(parts[0])
            else: linear, angular = float(parts[0]), float(parts[1])
            motion.set_manual_velocity(linear, angular)
        except ValueError: print(f"TURN 명령 파싱 실패: {line}")
    else: print(f"서버 메시지: {line}")

def receive_async(sock: socket.socket, motion: MotionController, arduino: ArduinoBridge) -> None:
    """소켓 수신 스레드: 줄 단위 명령을 읽어 콜백으로 넘긴다."""
    buffer = bytearray()
    while True:
        try:
            chunk = sock.recv(1024)
            if not chunk: print("서버와의 연결이 끊어졌습니다."); break
            buffer.extend(chunk)
            while b"\n" in buffer:
                raw, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                line = raw.decode("utf-8", errors="ignore").strip()
                if line: handle_server_message(line, motion, arduino)
        except OSError: print("메시지 수신 스레드 오류."); break

def send_control_message(sock: socket.socket, message: str) -> None:
    """영상 스트림과 동일한 채널로 제어 메시지를 보낸다."""
    if not message.strip(): return
    payload = (message.strip() + "\n").encode("utf-8")
    header = struct.pack(">L", 0)
    try:
        sock.sendall(header + payload)
        print(f"[Pi → Server]: {message.strip()}")
    except OSError as exc:
        print(f"제어 메시지 전송 실패: {exc}")

def main() -> None:
    """TurtleBot 클라이언트를 초기화하고 영상·센서 스트림을 전송한다."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.connect(SERVER_ADDR)
        print(f"서버 {SERVER_ADDR[0]}:{SERVER_ADDR[1]}에 연결되었습니다.")
        if not authenticate(sock): raise ConnectionRefusedError("서버 인증 실패")
        print("서버 인증을 통과했습니다.")

        stop_event = threading.Event()
        odom_bridge = OdometryBridge()
        if not odom_bridge.start(): raise RuntimeError("OdometryBridge 시작 실패")
        
        motion = MotionController(odom_bridge, lambda msg: send_control_message(sock, msg))
        arduino_bridge = ArduinoBridge(SERIAL_PORT, SERIAL_BAUDRATE, lambda msg: send_control_message(sock, msg))
        arduino_bridge.start()

        threading.Thread(target=receive_async, args=(sock, motion, arduino_bridge), daemon=True).start()
        
        cap_info = configure_capture()
        if not cap_info: raise RuntimeError("카메라 설정 실패")
        cap, sensor_fps = cap_info

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        frame_interval = 1.0 / min(TARGET_FPS, sensor_fps) if TARGET_FPS > 0 and sensor_fps > 0 else 0.0

        next_frame_time = time.perf_counter()
        next_pose_send = time.perf_counter()
        while not stop_event.is_set():
            cap.grab()
            ret, frame = cap.read()
            if not ret: continue

            if frame.shape[1] != FRAME_SIZE[0] or frame.shape[0] != FRAME_SIZE[1]:
                frame = cv2.resize(frame, FRAME_SIZE, interpolation=cv2.INTER_LINEAR)

            ok, encoded = cv2.imencode(".jpg", frame, encode_param)
            if not ok: continue

            payload = encoded.tobytes()
            header = struct.pack(">L", len(payload))
            sock.sendall(header + payload)

            now = time.perf_counter()
            if now >= next_pose_send:
                if odom_bridge.has_pose():
                    pose_world = odom_bridge.get_pose(relative=False)
                    rel_x, rel_y = odom_bridge.to_relative(pose_world[0], pose_world[1])
                    pose = (rel_x, rel_y, pose_world[2])
                    header = struct.pack(">L", 0)
                    payload = f"POSE:{pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f}\n".encode("utf-8")
                    sock.sendall(header + payload)
                next_pose_send = now + POSE_SEND_INTERVAL

            if frame_interval > 0:
                sleep_for = next_frame_time - time.perf_counter()
                if sleep_for > 0: time.sleep(sleep_for)
                next_frame_time += frame_interval

    except (KeyboardInterrupt, ConnectionRefusedError, ConnectionAbortedError) as e:
        print(f"클라이언트 종료: {e}")
    finally:
        stop_event.set()
        if 'motion' in locals(): motion.stop()
        if 'odom_bridge' in locals(): odom_bridge.stop()
        if 'arduino_bridge' in locals(): arduino_bridge.stop()
        if 'cap' in locals() and cap.isOpened(): cap.release()
        sock.close()
        print("클라이언트 리소스 정리 완료.")

if __name__ == "__main__":
    main()

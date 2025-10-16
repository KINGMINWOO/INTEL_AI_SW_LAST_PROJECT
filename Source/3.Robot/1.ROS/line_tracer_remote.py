#!/usr/bin/env python3
import math

import rclpy
import os
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class LineTracerRemote(Node):
    def __init__(self):
        super().__init__('line_tracer_remote')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Int32MultiArray, '/line_sensor', self.sensor_callback, 10)
        self._shutdown_requested = False
        self._shutdown_timer = self.create_timer(0.1, self._handle_shutdown_request)

        # === [추가된 설정] 좌우 회전 각도 ===
        self._left_sweep_angle = math.radians(100)   # 왼쪽 100도
        self._right_sweep_angle = math.radians(45)   # 오른쪽 45도

        self._scan_angular_speed = 0.6               # 회전 속도 (rad/s)
        self._calibration_active = True
        self._calibration_direction = 1.0            # +1: 왼쪽, -1: 오른쪽
        self._calibration_remaining = self._left_sweep_angle
        self._last_calibration_time = self.get_clock().now()
        self._calibration_timer = self.create_timer(0.05, self._handle_calibration_step)
        self._last_calibration_sensor = None

        self.linear_speed = 0.08
        self.angular_speed = 0.2
        self.get_logger().info("💻 Remote Line Tracer Started!")
        self.get_logger().info("라인 추적 준비: 센서 [1,1]을 찾기 위해 좌/우 회전을 시작합니다.")

    def sensor_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn("라인 센서 메시지 형식이 올바르지 않습니다. 최소 2개의 값을 기대합니다.")
            return

        raw_left, raw_right = msg.data[:2]
        if self._calibration_active:
            self._process_calibration_reading(raw_left, raw_right)
            return

        if raw_left == 0 and raw_right == 0:
            stop_twist = Twist()
            self.cmd_pub.publish(stop_twist)
            self.get_logger().info("라인 트레이서 종료 신호 수신 → 노드 종료를 요청합니다.")
            self._shutdown_requested = True
            return

        L, R = raw_left, raw_right
        twist = Twist()
        L = 1 - L
        R = 1 - R
        self.get_logger().info(f"센서 수신 → L={L}, R={R}")

        if L == 0 and R == 0:
            twist.linear.x = self.linear_speed
        elif L == 0 and R == 1:
            twist.angular.z = self.angular_speed
        elif L == 1 and R == 0:
            twist.angular.z = -self.angular_speed
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def _handle_shutdown_request(self):
        if not self._shutdown_requested:
            return
        self.get_logger().info("라인 트레이서를 종료합니다.")
        self._shutdown_requested = False
        self._shutdown_timer.cancel()
        self.destroy_node()
        rclpy.shutdown()
        os._exit(0)

    # === 🔸 수정된 회전 단계 로직 ===
    def _handle_calibration_step(self):
        if not self._calibration_active:
            return

        now = self.get_clock().now()
        delta = (now - self._last_calibration_time).nanoseconds / 1e9
        self._last_calibration_time = now
        self._calibration_remaining -= abs(self._scan_angular_speed) * delta

        twist = Twist()
        twist.angular.z = self._calibration_direction * self._scan_angular_speed
        self.cmd_pub.publish(twist)

        # 회전 각도 제한 도달 시 방향 전환
        if self._calibration_remaining <= 0.0:
            if self._calibration_direction > 0:
                # 왼쪽 스캔 완료 → 오른쪽으로 전환 (45도)
                self._calibration_direction = -1.0
                self._calibration_remaining = self._right_sweep_angle
                self.get_logger().info("라인 추적 준비: 오른쪽으로 회전 (45도)")
            else:
                # 오른쪽 스캔 완료 → 왼쪽으로 전환 (100도)
                self._calibration_direction = 1.0
                self._calibration_remaining = self._left_sweep_angle
                self.get_logger().info("라인 추적 준비: 왼쪽으로 회전 (100도)")

    def _process_calibration_reading(self, raw_left: int, raw_right: int) -> None:
        current = (raw_left, raw_right)
        if current != self._last_calibration_sensor:
            self.get_logger().info(f"스캔 중 센서 수신 → raw L={raw_left}, R={raw_right}")
            self._last_calibration_sensor = current
        if raw_left == 1 and raw_right == 1:
            self.get_logger().info("센서 [1,1] 감지 → 라인 추적 준비 완료.")
            self._finish_calibration()

    def _finish_calibration(self) -> None:
        if not self._calibration_active:
            return
        self._calibration_active = False
        self._last_calibration_sensor = None
        if self._calibration_timer is not None:
            self._calibration_timer.cancel()
            self._calibration_timer = None
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        self.get_logger().info("라인 추적 준비 단계가 종료되었습니다. 라인 트레이싱을 시작합니다.")


def main(args=None):
    rclpy.init(args=args)
    node = LineTracerRemote()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

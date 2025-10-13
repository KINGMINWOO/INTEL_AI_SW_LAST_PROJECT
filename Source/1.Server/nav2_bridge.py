"""Nav2 NavigateToPose 액션을 간단히 사용하기 위한 브리지 모듈."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """단일 yaw(라디안)을 geometry_msgs/Quaternion으로 변환."""
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


@dataclass
class Nav2Result:
    success: bool
    status: int
    message: str


STATUS_TEXT = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class Nav2Bridge(Node):
    """Nav2 NavigateToPose 액션을 동기 호출 형태로 감싸는 헬퍼."""

    def __init__(self, action_name: str = "navigate_to_pose", *, executor: Optional[MultiThreadedExecutor] = None) -> None:
        super().__init__("nav2_bridge")
        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self._executor = executor or MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._goal_lock = threading.Lock()
        self._current_goal_handle: Optional[object] = None

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 NavigateToPose 액션 서버를 찾지 못했습니다.")
            raise RuntimeError("NAV2_SERVER_UNAVAILABLE")
        self.get_logger().info("Nav2 NavigateToPose 액션 서버 연결 완료")

    def navigate(self, x: float, y: float, yaw: float = 0.0, frame_id: str = "map",
                 timeout: float = 120.0) -> Nav2Result:
        """주어진 좌표와 yaw로 이동을 요청하고 결과를 동기적으로 반환."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = yaw_to_quaternion(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        complete_event = threading.Event()
        result_holder: dict[str, Nav2Result] = {}

        def _result_callback(future) -> None:
            try:
                result = future.result()
            except Exception as exc:  # noqa: BLE001
                result_holder["value"] = Nav2Result(False, GoalStatus.STATUS_UNKNOWN, str(exc))
                complete_event.set()
                return

            status = result.status
            success = status == GoalStatus.STATUS_SUCCEEDED
            status_msg = STATUS_TEXT.get(status, "UNKNOWN")
            result_holder["value"] = Nav2Result(success, status, status_msg)
            with self._goal_lock:
                self._current_goal_handle = None
            complete_event.set()

        def _goal_response_callback(future) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                result_holder["value"] = Nav2Result(False, GoalStatus.STATUS_REJECTED, "GOAL_REJECTED")
                complete_event.set()
                return
            with self._goal_lock:
                self._current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_result_callback)

        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(_goal_response_callback)

        finished = complete_event.wait(timeout)
        if not finished:
            self.get_logger().warning("Nav2 목표 수행이 타임아웃 되었습니다. 취소를 요청합니다.")
            try:
                with self._goal_lock:
                    goal_handle = self._current_goal_handle
                if goal_handle is not None:
                    cancel_future = goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(lambda _: None)
            except Exception:  # noqa: BLE001
                pass
            return Nav2Result(False, GoalStatus.STATUS_ABORTED, "TIMEOUT")

        return result_holder.get("value", Nav2Result(False, GoalStatus.STATUS_UNKNOWN, "NO_RESULT"))

    def shutdown(self) -> None:
        """스레드와 rclpy 리소스를 정리."""
        self._executor.shutdown()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self.destroy_node()

    def cancel_current_goal(self, wait_timeout: float = 5.0) -> bool:
        """현재 진행 중인 목표가 있다면 취소를 요청."""
        with self._goal_lock:
            goal_handle = self._current_goal_handle
        if goal_handle is None:
            return False

        cancel_event = threading.Event()

        def _cancel_done(_):
            cancel_event.set()

        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(_cancel_done)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Nav2 목표 취소 실패: {exc}")
            return False

        cancelled = cancel_event.wait(wait_timeout)
        if cancelled:
            with self._goal_lock:
                self._current_goal_handle = None
        return cancelled


def create_nav2_bridge() -> Nav2Bridge:
    """rclpy 컨텍스트를 초기화하고 Nav2 브리지를 생성."""
    if not rclpy.ok():
        rclpy.init()
    executor = MultiThreadedExecutor()
    return Nav2Bridge(executor=executor)

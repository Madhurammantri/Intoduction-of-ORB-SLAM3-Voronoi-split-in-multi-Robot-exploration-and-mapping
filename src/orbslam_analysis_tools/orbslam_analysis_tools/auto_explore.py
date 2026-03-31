#!/usr/bin/env python3
"""
auto_explore.py

Single-robot autonomous frontier exploration (simple + thesis friendly)

Fixes compared to old version:
- Prints total exploration time
- Stops automatically when exploration is done (no frontiers for a while)
- Avoids repeating the same goal points near the end
"""

import math
import time
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros


def dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


class AutoExplorer(Node):
    """
    Simple frontier explorer for ONE robot namespace.
    """

    def __init__(self):
        super().__init__('auto_explorer')

        # ---------------- Parameters ----------------
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('replan_period', 2.0)
        self.declare_parameter('stride', 4)                  # scan map every N cells (faster)
        self.declare_parameter('min_goal_separation', 0.75)  # don't re-pick almost same goal
        self.declare_parameter('goal_timeout_sec', 120.0)    # if nav gets stuck, pick new goal

        # "done" condition: if no frontiers for N cycles -> stop
        self.declare_parameter('no_frontier_cycles_to_finish', 15)

        self.robot = str(self.get_parameter('robot_name').value)
        self.replan_period = float(self.get_parameter('replan_period').value)
        self.stride = int(self.get_parameter('stride').value)
        self.min_sep = float(self.get_parameter('min_goal_separation').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.finish_cycles = int(self.get_parameter('no_frontier_cycles_to_finish').value)

        # ---------------- Topics / Frames ----------------
        self.map_topic = f'/{self.robot}/map'
        self.map_frame = f'{self.robot}/map'
        self.base_frame = f'{self.robot}/base_link'
        self.nav_action = f'/{self.robot}/navigate_to_pose'

        self.get_logger().info(f'🚀 Auto exploration started for {self.robot}')
        self.get_logger().info(f'   map: {self.map_topic}')
        self.get_logger().info(f'   nav: {self.nav_action}')

        # ---------------- State ----------------
        self.map: Optional[OccupancyGrid] = None
        self.goal_active = False
        self.goal_sent_time = 0.0
        self.start_time = time.time()

        self.last_goal: Optional[Tuple[float, float]] = None
        self.recent_goals: List[Tuple[float, float]] = []  # small rolling memory
        self.max_recent = 50

        self.no_frontier_cycles = 0
        self.finished = False

        # ---------------- Map subscription ----------------
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, qos)

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- Nav2 action ----------------
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)

        # ---------------- Timer loop ----------------
        self.timer = self.create_timer(self.replan_period, self.loop)

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def loop(self):
        if self.finished:
            return

        # If goal is active, only watch timeout
        if self.goal_active:
            if (time.time() - self.goal_sent_time) > self.goal_timeout:
                self.get_logger().warn("⏱️ Goal timeout. Picking a new goal.")
                self.goal_active = False
            return

        # Need map
        if self.map is None:
            self.get_logger().warn("Waiting for map...")
            return

        # Need Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Waiting for Nav2 action server...")
            return

        # Need TF
        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            self.get_logger().warn("Waiting for TF...")
            return

        # Find a good frontier
        goal = self.pick_frontier(robot_xy)
        if goal is None:
            self.no_frontier_cycles += 1
            self.get_logger().warn(
                f"No frontiers found ({self.no_frontier_cycles}/{self.finish_cycles})"
            )

            if self.no_frontier_cycles >= self.finish_cycles:
                self.finish_exploration()
            return

        # Reset "no frontier" counter when we have a target
        self.no_frontier_cycles = 0
        self.send_goal(goal, robot_xy)

    def finish_exploration(self):
        if self.finished:
            return
        self.finished = True

        elapsed = time.time() - self.start_time
        self.get_logger().info("✅ Exploration complete (frontiers exhausted).")
        self.get_logger().info(f"⏱️ Total exploration time: {elapsed:.1f} sec")

        # stop timer so it doesn't keep printing
        self.timer.cancel()

        # shutdown cleanly (avoid double-shutdown errors)
        self.create_timer(0.2, self._shutdown_once)

    def _shutdown_once(self):
        # this timer callback might run more than once; guard it
        self.timer.cancel()  # safe even if already cancelled
        if rclpy.ok():
            rclpy.shutdown()

    def get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            return (
                tf.transform.translation.x,
                tf.transform.translation.y
            )
        except Exception:
            return None

    def pick_frontier(self, robot_xy: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        frontiers = self.find_frontiers()
        if not frontiers:
            return None

        # Filter: don't pick almost-same-as-last or very recent goals
        valid = []
        for f in frontiers:
            if self.last_goal and dist2(f, self.last_goal) < self.min_sep ** 2:
                continue
            too_close_recent = any(dist2(f, g) < (self.min_sep ** 2) for g in self.recent_goals)
            if too_close_recent:
                continue
            valid.append(f)

        if not valid:
            return None

        # Choose nearest valid frontier (simple + stable)
        valid.sort(key=lambda p: dist2(p, robot_xy))
        return valid[0]

    def find_frontiers(self) -> List[Tuple[float, float]]:
        grid = self.map
        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        data = grid.data

        def idx(x: int, y: int) -> int:
            return y * w + x

        frontiers: List[Tuple[float, float]] = []

        # Scan every "stride" cells for speed
        for y in range(1, h - 1, self.stride):
            for x in range(1, w - 1, self.stride):
                if data[idx(x, y)] != -1:  # only unknown cells
                    continue

                # unknown cell adjacent to free cell => frontier
                if (
                    data[idx(x + 1, y)] == 0 or
                    data[idx(x - 1, y)] == 0 or
                    data[idx(x, y + 1)] == 0 or
                    data[idx(x, y - 1)] == 0
                ):
                    wx = ox + (x + 0.5) * res
                    wy = oy + (y + 0.5) * res
                    frontiers.append((wx, wy))

        return frontiers

    def send_goal(self, goal_xy: Tuple[float, float], robot_xy: Tuple[float, float]):
        gx, gy = goal_xy
        rx, ry = robot_xy
        yaw = math.atan2(gy - ry, gx - rx)

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose

        self.get_logger().info(f"➡️ {self.robot} exploring to ({gx:.2f}, {gy:.2f})")

        self.goal_active = True
        self.goal_sent_time = time.time()
        self.last_goal = goal_xy
        self.recent_goals.append(goal_xy)
        self.recent_goals = self.recent_goals[-self.max_recent:]

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.goal_active = False
            self.get_logger().warn("Goal rejected")
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self.goal_result)

    def goal_result(self, future):
        # We keep it simple: any result means "goal finished"
        self.goal_active = False
        self.get_logger().info("🏁 Goal finished")


def main():
    rclpy.init()
    node = AutoExplorer()
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


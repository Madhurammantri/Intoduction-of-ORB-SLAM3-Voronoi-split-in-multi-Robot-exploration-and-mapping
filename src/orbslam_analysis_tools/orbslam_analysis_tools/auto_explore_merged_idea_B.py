#!/usr/bin/env python3
"""
auto_explore_merged_idea_B.py

Merged occupancy grid exploration (/map) + shared claims (/claimed_goals).
Region split Idea-B: X-axis partition based on robot initial poses:
- Sort robots by initial x
- Compute midpoints between adjacent robots
- Each robot only explores frontiers whose x lies in its partition

This is simple, stable, thesis-friendly.
"""

import math
import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros


def d2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def parse_peers(peers_str: str) -> List[Tuple[str, float, float]]:
    peers = []
    peers_str = (peers_str or "").strip()
    if not peers_str:
        return peers
    for item in peers_str.split(";"):
        item = item.strip()
        if not item:
            continue
        name, xy = item.split(":")
        x_s, y_s = xy.split(",")
        peers.append((name.strip(), float(x_s), float(y_s)))
    return peers


class ExplorerIdeaB(Node):
    def __init__(self):
        super().__init__("auto_explore_merged_idea_B")

        self.declare_parameter("robot_name", "robot_1")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("nav_action", "")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "")

        self.declare_parameter("stride", 4)
        self.declare_parameter("cluster_radius", 0.8)
        self.declare_parameter("min_cluster_size", 8)

        self.declare_parameter("tick_sec", 2.0)
        self.declare_parameter("claim_topic", "/claimed_goals")
        self.declare_parameter("claim_radius", 1.2)
        self.declare_parameter("min_goal_separation", 0.8)
        self.declare_parameter("goal_timeout_sec", 180.0)

        self.declare_parameter("peers", "robot_1:0,0;robot_2:5,0")

        self.robot = self.get_parameter("robot_name").value
        self.map_topic = self.get_parameter("map_topic").value

        nav_action = self.get_parameter("nav_action").value
        self.nav_action = nav_action if nav_action else f"/{self.robot}/navigate_to_pose"

        self.global_frame = self.get_parameter("global_frame").value
        base_frame = self.get_parameter("base_frame").value
        self.base_frame = base_frame if base_frame else f"{self.robot}/base_link"

        self.stride = int(self.get_parameter("stride").value)
        self.cluster_radius = float(self.get_parameter("cluster_radius").value)
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)

        self.tick_sec = float(self.get_parameter("tick_sec").value)
        self.claim_topic = self.get_parameter("claim_topic").value
        self.claim_radius = float(self.get_parameter("claim_radius").value)
        self.min_goal_sep = float(self.get_parameter("min_goal_separation").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)

        self.peers = parse_peers(self.get_parameter("peers").value)
        if not self.peers:
            self.peers = [(self.robot, 0.0, 0.0)]

        self.map_msg: Optional[OccupancyGrid] = None
        self.claims: List[Tuple[float, float]] = []
        self.last_goal: Optional[Tuple[float, float]] = None

        self.goal_active = False
        self.goal_sent_time = 0.0

        self.no_frontier_ticks = 0
        self.no_frontier_ticks_to_stop = 10

        self.start_time = time.time()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ROS
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.claim_pub = self.create_publisher(PoseStamped, self.claim_topic, qos)
        self.create_subscription(PoseStamped, self.claim_topic, self.claim_cb, qos)

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)
        self.timer = self.create_timer(self.tick_sec, self.tick)

        self.get_logger().info(f"🚀 Auto exploration (IDEA B: X-partition) started for {self.robot}")

    def map_cb(self, msg):
        self.map_msg = msg

    def claim_cb(self, msg):
        self.claims.append((msg.pose.position.x, msg.pose.position.y))
        self.claims = self.claims[-400:]

    def tick(self):
        if self.goal_active:
            if (time.time() - self.goal_sent_time) > self.goal_timeout:
                self.get_logger().warn("⏱️ Goal timeout -> releasing goal")
                self.goal_active = False
            return

        if self.map_msg is None:
            self.get_logger().warn("Waiting for merged map...")
            return

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            self.get_logger().warn("Waiting for TF...")
            return

        pts = self.find_frontier_points(self.map_msg)
        if not pts:
            self.no_frontier_ticks += 1
            if self.no_frontier_ticks >= self.no_frontier_ticks_to_stop:
                self.finish()
            return
        self.no_frontier_ticks = 0

        clusters = self.cluster_points(pts, self.cluster_radius)
        clusters = [c for c in clusters if len(c) >= self.min_cluster_size]
        if not clusters:
            self.no_frontier_ticks += 1
            if self.no_frontier_ticks >= self.no_frontier_ticks_to_stop:
                self.finish()
            return

        goal = self.pick_goal(clusters, robot_xy)
        if goal is None:
            self.no_frontier_ticks += 1
            if self.no_frontier_ticks >= self.no_frontier_ticks_to_stop:
                self.finish()
            return

        self.send_goal(goal)

    def finish(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info("✅ Exploration complete")
        self.get_logger().info(f"⏱️ Total exploration time: {elapsed:.1f} sec")
        try:
            self.timer.cancel()
        except Exception:
            pass

    def find_frontier_points(self, grid):
        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        data = grid.data

        def idx(x, y):
            return y * w + x

        pts = []
        for y in range(1, h - 1, self.stride):
            for x in range(1, w - 1, self.stride):
                if data[idx(x, y)] != -1:
                    continue
                if (data[idx(x + 1, y)] == 0 or data[idx(x - 1, y)] == 0 or
                        data[idx(x, y + 1)] == 0 or data[idx(x, y - 1)] == 0):
                    pts.append((ox + (x + 0.5) * res, oy + (y + 0.5) * res))
        return pts

    def cluster_points(self, pts, radius):
        r2 = radius * radius
        used = [False] * len(pts)
        clusters = []

        for i in range(len(pts)):
            if used[i]:
                continue
            used[i] = True
            q = [i]
            c = [pts[i]]
            while q:
                k = q.pop()
                for j in range(len(pts)):
                    if used[j]:
                        continue
                    if d2(pts[k], pts[j]) <= r2:
                        used[j] = True
                        q.append(j)
                        c.append(pts[j])
            clusters.append(c)
        return clusters

    def centroid(self, cluster):
        sx = sum(p[0] for p in cluster)
        sy = sum(p[1] for p in cluster)
        n = max(1, len(cluster))
        return (sx / n, sy / n)

    def my_x_bounds(self) -> Tuple[float, float]:
        # sort by initial x
        peers = sorted(self.peers, key=lambda p: p[1])  # (name,x,y)
        xs = [p[1] for p in peers]
        names = [p[0] for p in peers]
        i = names.index(self.robot) if self.robot in names else 0

        # midpoints define partitions
        left = -1e9
        right = 1e9
        if i > 0:
            left = 0.5 * (xs[i - 1] + xs[i])
        if i < len(xs) - 1:
            right = 0.5 * (xs[i] + xs[i + 1])
        return left, right

    def is_claimed(self, p):
        r2 = self.claim_radius * self.claim_radius
        return any(d2(p, c) <= r2 for c in self.claims)

    def pick_goal(self, clusters, robot_xy):
        left, right = self.my_x_bounds()

        cand = []
        for c in clusters:
            cen = self.centroid(c)

            # only frontiers in my x-partition
            if not (left <= cen[0] <= right):
                continue

            if self.last_goal and d2(cen, self.last_goal) < self.min_goal_sep * self.min_goal_sep:
                continue
            if self.is_claimed(cen):
                continue

            dist = math.sqrt(d2(cen, robot_xy))
            size = len(c)
            score = dist - 0.15 * size
            cand.append((score, cen))

        if not cand:
            return None
        cand.sort(key=lambda x: x[0])
        return cand[0][1]

    def get_robot_xy(self):
        for base in [self.base_frame, "base_link"]:
            try:
                t = self.tf_buffer.lookup_transform(self.global_frame, base, rclpy.time.Time())
                return (t.transform.translation.x, t.transform.translation.y)
            except Exception:
                continue
        return None

    def send_goal(self, xy):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return

        claim = PoseStamped()
        claim.header.frame_id = self.global_frame
        claim.header.stamp = self.get_clock().now().to_msg()
        claim.pose.position.x = float(xy[0])
        claim.pose.position.y = float(xy[1])
        claim.pose.orientation.w = 1.0
        self.claim_pub.publish(claim)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"➡️ {self.robot} → ({xy[0]:.2f}, {xy[1]:.2f})")
        self.goal_active = True
        self.goal_sent_time = time.time()
        self.last_goal = xy

        self.nav_client.send_goal_async(goal).add_done_callback(self._resp)

    def _resp(self, future):
        try:
            h = future.result()
        except Exception:
            self.goal_active = False
            return
        if not h.accepted:
            self.goal_active = False
            return
        h.get_result_async().add_done_callback(self._done)

    def _done(self, future):
        self.get_logger().info("🏁 Goal finished")
        self.goal_active = False


def main():
    rclpy.init()
    node = ExplorerIdeaB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


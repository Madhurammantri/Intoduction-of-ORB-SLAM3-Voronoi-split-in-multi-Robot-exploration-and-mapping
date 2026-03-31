#!/usr/bin/env python3
"""
auto_explore_merged_my_idea.py

Robust frontier explorer for Nav2 (multi-robot compatible).

Key fixes in this version:
- Subscribes to /map with TRANSIENT_LOCAL QoS (required for map_merge / slam_toolbox latched maps)
- merged_mode and use_claims accept BOOL overrides (merged_mode:=true works)
- Optional stop_when_merge_satisfied:
    Stops ONLY when:
      (A) no frontiers for N ticks
      (B) merged TF stable for merged_tf_stable_sec
      (C) merged /map has become "stable" (change ratio <= threshold) for map_stable_sec
      (D) min_runtime_sec elapsed
- In merged_mode, can compute goals in global_frame (world) but send Nav2 goals in nav_goal_frame (robot_X/map)
  by transforming (world -> robot_X/map). This prevents ABORTED/204 due to frame mismatch.
"""

import math
import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterNotDeclaredException

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros


def d2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def parse_peer_names(s: str) -> List[str]:
    s = (s or "").strip()
    if not s:
        return []
    return [x.strip() for x in s.split(",") if x.strip()]


def _parse_bool_like(v, default=None):
    """Accept bool or string-y bools. If cannot parse -> default."""
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        x = v.strip().lower()
        if x in ("true", "1", "yes", "y", "on"):
            return True
        if x in ("false", "0", "no", "n", "off"):
            return False
    return default


def quat_to_yaw(qx, qy, qz, qw):
    # yaw from quaternion
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    s = 2.0 * (qw * qz + qx * qy)
    c = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(s, c)


def rotate2d(x, y, yaw):
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (cy * x - sy * y, sy * x + cy * y)


class ExplorerMyIdea(Node):
    def __init__(self):
        super().__init__("auto_explore_merged_my_idea")

        dyn = ParameterDescriptor(dynamic_typing=True)

        # ----------------------------
        # Basic parameters
        # ----------------------------
        self.declare_parameter("robot_name", "robot_1")
        self.declare_parameter("robots", "robot_1,robot_2")  # order matters
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("nav_action", "")

        # Frames
        self.declare_parameter("global_frame", "auto")  # auto -> map header frame
        self.declare_parameter("base_frame", "")

        # merged behavior (dynamic typing so merged_mode:=true works)
        self.declare_parameter("merged_mode", "auto", dyn)   # "auto" | bool
        # nav goal frame: where NavigateToPose goal is sent
        # In multi-robot Nav2, this is usually robot_X/map.
        self.declare_parameter("nav_goal_frame", "auto")

        # frontier detection
        self.declare_parameter("stride", 2)
        self.declare_parameter("cluster_radius", 0.8)
        self.declare_parameter("min_cluster_size", 6)

        # timing
        self.declare_parameter("tick_sec", 2.0)
        self.declare_parameter("goal_timeout_sec", 180.0)

        # filtering (BASE values)
        self.declare_parameter("min_goal_distance", 0.9)
        self.declare_parameter("min_goal_separation", 0.8)

        # k-means
        self.declare_parameter("kmeans_iters", 8)

        # stop (frontier-based)
        self.declare_parameter("no_frontier_ticks_to_stop", 15)
        self.declare_parameter("require_merged_tf_to_stop", True)
        self.declare_parameter("merged_tf_stable_sec", 8.0)

        # stop (merge-satisfied)
        self.declare_parameter("stop_when_merge_satisfied", False)
        self.declare_parameter("merge_min_runtime_sec", 60.0)
        self.declare_parameter("map_stable_sec", 25.0)
        self.declare_parameter("map_change_ratio_threshold", 0.001)  # 0.1% sampled diff
        self.declare_parameter("map_change_sample_step", 11)         # sample every N cells (speed)

        # fallback
        self.declare_parameter("fallback_to_any_bucket", True)

        # ----------------------------
        # Robustness / Anti-stuck
        # ----------------------------
        self.declare_parameter("adaptive_relax", True)
        self.declare_parameter("relax_after_no_candidate_ticks", 2)

        self.declare_parameter("min_goal_distance_floor", 0.15)
        self.declare_parameter("min_cluster_size_floor", 1)
        self.declare_parameter("min_goal_separation_floor", 0.15)

        self.declare_parameter("track_failed_goals", True)
        self.declare_parameter("failed_goal_ttl_sec", 60.0)
        self.declare_parameter("failed_goal_radius", 0.6)
        self.declare_parameter("max_failed_goals", 300)

        self.declare_parameter("last_goal_cooldown_sec", 8.0)

        # ----------------------------
        # Read early params
        # ----------------------------
        self.robot = self.get_parameter("robot_name").value
        self.robots = parse_peer_names(self.get_parameter("robots").value)
        if not self.robots:
            self.robots = [self.robot]

        self.map_topic = self.get_parameter("map_topic").value
        nav_action = self.get_parameter("nav_action").value
        self.nav_action = nav_action if nav_action else f"/{self.robot}/navigate_to_pose"

        mm_raw = self.get_parameter("merged_mode").value
        mm_bool = _parse_bool_like(mm_raw, default=None)
        if mm_bool is not None:
            self.merged_mode = mm_bool
        else:
            # "auto" behavior
            mm_str = (str(mm_raw) if mm_raw is not None else "auto").strip().lower()
            if mm_str in ("true", "1", "yes", "y"):
                self.merged_mode = True
            elif mm_str in ("false", "0", "no", "n"):
                self.merged_mode = False
            else:
                self.merged_mode = (self.map_topic == "/map")

        # ----------------------------
        # Claims (dynamic typing so use_claims:=false works)
        # ----------------------------
        self.declare_parameter("claim_topic", "/claimed_goals")
        self.declare_parameter("claim_radius", 1.2)
        self.declare_parameter("use_claims", bool(self.merged_mode), dyn)
        self.declare_parameter("claim_ttl_sec", 30.0)

        # ----------------------------
        # Read remaining params
        # ----------------------------
        self.global_frame_param = self.get_parameter("global_frame").value
        self.global_frame: Optional[str] = None

        base_frame = self.get_parameter("base_frame").value
        self.base_frame = base_frame if base_frame else f"{self.robot}/base_link"

        nav_goal_frame = (self.get_parameter("nav_goal_frame").value or "").strip()
        self.nav_goal_frame_param = nav_goal_frame if nav_goal_frame else "auto"
        self.nav_goal_frame: Optional[str] = None

        self.stride = int(self.get_parameter("stride").value)
        self.cluster_radius = float(self.get_parameter("cluster_radius").value)
        self.min_cluster_size_base = int(self.get_parameter("min_cluster_size").value)

        self.tick_sec = float(self.get_parameter("tick_sec").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)

        self.min_goal_dist_base = float(self.get_parameter("min_goal_distance").value)
        self.min_goal_sep_base = float(self.get_parameter("min_goal_separation").value)

        self.kmeans_iters = int(self.get_parameter("kmeans_iters").value)

        self.no_frontier_ticks_to_stop = int(self.get_parameter("no_frontier_ticks_to_stop").value)
        self.require_merged_tf_to_stop = bool(self.get_parameter("require_merged_tf_to_stop").value)
        self.merged_tf_stable_sec = float(self.get_parameter("merged_tf_stable_sec").value)

        self.stop_when_merge_satisfied = bool(self.get_parameter("stop_when_merge_satisfied").value)
        self.merge_min_runtime_sec = float(self.get_parameter("merge_min_runtime_sec").value)
        self.map_stable_sec = float(self.get_parameter("map_stable_sec").value)
        self.map_change_ratio_threshold = float(self.get_parameter("map_change_ratio_threshold").value)
        self.map_change_sample_step = int(self.get_parameter("map_change_sample_step").value)

        self.fallback_to_any_bucket = bool(self.get_parameter("fallback_to_any_bucket").value)

        self.adaptive_relax = bool(self.get_parameter("adaptive_relax").value)
        self.relax_after_ticks = int(self.get_parameter("relax_after_no_candidate_ticks").value)

        self.min_goal_dist_floor = float(self.get_parameter("min_goal_distance_floor").value)
        self.min_cluster_size_floor = int(self.get_parameter("min_cluster_size_floor").value)
        self.min_goal_sep_floor = float(self.get_parameter("min_goal_separation_floor").value)

        self.track_failed = bool(self.get_parameter("track_failed_goals").value)
        self.failed_ttl = float(self.get_parameter("failed_goal_ttl_sec").value)
        self.failed_radius = float(self.get_parameter("failed_goal_radius").value)
        self.max_failed = int(self.get_parameter("max_failed_goals").value)

        self.last_goal_cooldown = float(self.get_parameter("last_goal_cooldown_sec").value)

        self.claim_topic = self.get_parameter("claim_topic").value
        self.claim_radius = float(self.get_parameter("claim_radius").value)

        uc_raw = self.get_parameter("use_claims").value
        uc_bool = _parse_bool_like(uc_raw, default=bool(self.merged_mode))
        self.use_claims = bool(uc_bool)

        self.claim_ttl = float(self.get_parameter("claim_ttl_sec").value)

        # ----------------------------
        # Adaptive current values
        # ----------------------------
        self.min_goal_dist = self.min_goal_dist_base
        self.min_goal_sep = self.min_goal_sep_base
        self.min_cluster_size = self.min_cluster_size_base

        # ----------------------------
        # State
        # ----------------------------
        self.map_msg: Optional[OccupancyGrid] = None

        self.claims: List[Tuple[float, float, float]] = []   # (x,y,t)
        self.failed: List[Tuple[float, float, float]] = []   # (x,y,t)

        self.last_goal: Optional[Tuple[float, float]] = None
        self.last_goal_time = 0.0

        self.goal_active = False
        self.goal_sent_time = 0.0
        self.goal_xy_sent: Optional[Tuple[float, float]] = None

        self.no_frontier_ticks = 0
        self.no_candidate_ticks = 0
        self.start_time = time.time()

        # merged TF stability
        self._merged_tf_is_stable = False
        self._merged_tf_first_ok_time: Optional[float] = None

        # map stability tracking (for stop_when_merge_satisfied)
        self._prev_map_sample: Optional[List[int]] = None
        self._map_stable_first_time: Optional[float] = None
        self._last_map_stable_log = 0.0

        # throttled logs
        self._last_wait_map_log = 0.0
        self._last_tf_warn = 0.0
        self._last_merge_warn = 0.0
        self._last_no_candidate_log = 0.0
        self._last_no_cluster_log = 0.0
        self._last_nav_server_warn = 0.0
        self._last_fallback_log = 0.0
        self._last_relax_log = 0.0
        self._last_nav_frame_warn = 0.0

        # ----------------------------
        # TF
        # ----------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------
        # ROS IO (IMPORTANT: map QoS must be TRANSIENT_LOCAL)
        # ----------------------------
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, map_qos)

        claim_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )

        if self.use_claims:
            self.claim_pub = self.create_publisher(PoseStamped, self.claim_topic, claim_qos)
            self.create_subscription(PoseStamped, self.claim_topic, self.claim_cb, claim_qos)
        else:
            self.claim_pub = None

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)
        self.timer = self.create_timer(self.tick_sec, self.tick)

        # ----------------------------
        # Startup logs
        # ----------------------------
        self.get_logger().info(f"🚀 Exploration (MY IDEA) started for {self.robot}")
        self.get_logger().info(f"   robots: {self.robots}")
        self.get_logger().info(f"   map_topic: {self.map_topic}")
        self.get_logger().info(f"   nav_action: {self.nav_action}")
        self.get_logger().info(f"   global_frame(param): {self.global_frame_param}")
        self.get_logger().info(f"   base_frame: {self.base_frame}")
        self.get_logger().info(f"   merged_mode: {self.merged_mode}")
        self.get_logger().info(f"   use_claims: {self.use_claims} (claim_ttl_sec={self.claim_ttl})")
        self.get_logger().info(f"   stop_when_merge_satisfied: {self.stop_when_merge_satisfied}")
        self.get_logger().info(f"   fallback_to_any_bucket: {self.fallback_to_any_bucket}")

    # ----------------------------
    # Callbacks
    # ----------------------------
    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

        if self.global_frame is None:
            if (self.global_frame_param or "").strip().lower() == "auto":
                self.global_frame = (msg.header.frame_id or "").strip() or "map"
            else:
                self.global_frame = (self.global_frame_param or "").strip() or "map"
            self.get_logger().info(
                f"✅ Resolved global_frame = {self.global_frame} (map header.frame_id={msg.header.frame_id})"
            )

        if self.nav_goal_frame is None:
            # auto behavior:
            # - if merged_mode, Nav2 per-robot usually expects robot_X/map goals
            # - else, send in global_frame
            if (self.nav_goal_frame_param or "").strip().lower() == "auto":
                self.nav_goal_frame = f"{self.robot}/map" if self.merged_mode else self.global_frame
            else:
                self.nav_goal_frame = self.nav_goal_frame_param
            self.get_logger().info(f"   nav_goal_frame: {self.nav_goal_frame}")

    def claim_cb(self, msg: PoseStamped):
        if not self.use_claims or self.global_frame is None:
            return
        if msg.header.frame_id != self.global_frame:
            return
        now = time.time()
        self.claims.append((msg.pose.position.x, msg.pose.position.y, now))
        self.claims = self.claims[-800:]

    # ----------------------------
    # Main loop
    # ----------------------------
    def tick(self):
        if self.goal_active:
            self.check_timeout()
            return

        if self.map_msg is None or self.global_frame is None or self.nav_goal_frame is None:
            now = time.time()
            if now - self._last_wait_map_log > 2.0:
                self.get_logger().warn(f"Waiting for map on {self.map_topic} ...")
                self._last_wait_map_log = now
            return

        self.update_merged_tf_stability()
        self.expire_lists()

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            now = time.time()
            if now - self._last_tf_warn > 2.0:
                self.get_logger().warn("Waiting for TF...")
                self._last_tf_warn = now
            return

        pts = self.find_frontier_points(self.map_msg)
        if not pts:
            self.no_frontier_ticks += 1
            self.no_candidate_ticks = 0
            self.reset_relax_to_base()
            self.maybe_stop()
            return

        self.no_frontier_ticks = 0

        clusters = self.cluster_points(pts, self.cluster_radius)
        clusters = [c for c in clusters if len(c) >= self.min_cluster_size]
        if not clusters:
            now = time.time()
            if now - self._last_no_cluster_log > 3.0:
                self.get_logger().warn("Frontiers exist but no clusters passed min_cluster_size yet")
                self._last_no_cluster_log = now
            self.no_candidate_ticks += 1
            self.maybe_adapt_relax()
            return

        # k-means assignment by cluster centroids
        cents = [self.centroid(c) for c in clusters]
        k = min(len(self.robots), len(cents))
        if k <= 0:
            self.no_candidate_ticks += 1
            self.maybe_adapt_relax()
            return

        centers, labels = self.kmeans_deterministic(cents, k, self.kmeans_iters)
        order = sorted(range(k), key=lambda i: (centers[i][0], centers[i][1]))
        robot_index = self.robots.index(self.robot) if self.robot in self.robots else 0
        my_bucket = order[min(robot_index, k - 1)]

        chosen = self.choose_goal(robot_xy, clusters, labels, my_bucket)

        if chosen is None:
            self.no_candidate_ticks += 1
            now = time.time()
            if now - self._last_no_candidate_log > 2.0:
                self.get_logger().warn(
                    "No valid goal candidates (too near / claimed / failed / filtered). "
                    "Explorer will relax filters automatically."
                )
                self._last_no_candidate_log = now

            if self.adaptive_relax and self.no_candidate_ticks >= self.relax_after_ticks:
                self.maybe_adapt_relax(force_log=True)
                clusters2 = [c for c in clusters if len(c) >= self.min_cluster_size]
                if clusters2:
                    chosen = self.choose_goal(robot_xy, clusters2, [0] * len(clusters2), None)

            if chosen is None:
                return
        else:
            self.no_candidate_ticks = 0
            self.soft_recover_to_base()

        _, goal_xy, dist, size = chosen
        self.get_logger().info(f"📌 pick goal dist={dist:.2f}m cluster={size} frame={self.global_frame}")
        self.send_goal(goal_xy)

    def choose_goal(self, robot_xy, clusters, labels, my_bucket):
        cand_bucket = self.build_candidates(robot_xy, clusters, labels, allowed_bucket=my_bucket)
        if cand_bucket:
            cand_bucket.sort(key=lambda x: x[0])
            return cand_bucket[0]

        if self.fallback_to_any_bucket:
            cand_all = self.build_candidates(robot_xy, clusters, labels, allowed_bucket=None)
            if cand_all:
                cand_all.sort(key=lambda x: x[0])
                now = time.time()
                if now - self._last_fallback_log > 2.0:
                    self.get_logger().warn(
                        "My k-means bucket has no valid candidates -> FALLBACK to best overall candidate."
                    )
                    self._last_fallback_log = now
                return cand_all[0]

        return None

    # ----------------------------
    # Adaptive relax / recover
    # ----------------------------
    def maybe_adapt_relax(self, force_log=False):
        if not self.adaptive_relax:
            return
        if self.no_candidate_ticks < self.relax_after_ticks and not force_log:
            return

        self.min_goal_dist = max(self.min_goal_dist_floor, self.min_goal_dist * 0.75)
        self.min_goal_sep = max(self.min_goal_sep_floor, self.min_goal_sep * 0.75)
        self.min_cluster_size = max(self.min_cluster_size_floor, int(math.floor(self.min_cluster_size * 0.7)))

        if self.last_goal and (time.time() - self.last_goal_time) > self.last_goal_cooldown:
            self.last_goal = None

        now = time.time()
        if force_log or (now - self._last_relax_log > 2.0):
            self.get_logger().warn(
                f"🧯 Relaxing filters: min_goal_distance={self.min_goal_dist:.2f}, "
                f"min_goal_separation={self.min_goal_sep:.2f}, min_cluster_size={self.min_cluster_size}"
            )
            self._last_relax_log = now

    def soft_recover_to_base(self):
        self.min_goal_dist = min(self.min_goal_dist_base, self.min_goal_dist * 1.05 + 0.01)
        self.min_goal_sep = min(self.min_goal_sep_base, self.min_goal_sep * 1.05 + 0.01)
        if self.min_cluster_size < self.min_cluster_size_base:
            self.min_cluster_size = min(self.min_cluster_size_base, self.min_cluster_size + 1)

    def reset_relax_to_base(self):
        self.min_goal_dist = self.min_goal_dist_base
        self.min_goal_sep = self.min_goal_sep_base
        self.min_cluster_size = self.min_cluster_size_base

    # ----------------------------
    # Candidate building + filters
    # ----------------------------
    def build_candidates(self, robot_xy, clusters, labels, allowed_bucket: Optional[int]):
        candidates = []
        now = time.time()

        for i, cluster in enumerate(clusters):
            if allowed_bucket is not None and i < len(labels) and labels[i] != allowed_bucket:
                continue

            goal_pt = min(cluster, key=lambda p: d2(p, robot_xy))
            dist = math.sqrt(d2(goal_pt, robot_xy))

            if dist < self.min_goal_dist:
                continue

            if self.last_goal is not None:
                if (now - self.last_goal_time) < self.last_goal_cooldown:
                    if d2(goal_pt, self.last_goal) < (self.min_goal_sep * self.min_goal_sep):
                        continue

            if self.use_claims and self.is_claimed(goal_pt):
                continue

            if self.track_failed and self.is_failed(goal_pt):
                continue

            size = len(cluster)
            score = dist - 0.10 * min(size, 200)
            candidates.append((score, goal_pt, dist, size))

        if candidates:
            candidates.sort(key=lambda x: x[0])
            best = candidates[0]
            if best[2] < 0.8 and len(candidates) > 3:
                far = max(candidates, key=lambda x: x[2])
                if far[2] > best[2] + 0.8:
                    return [best, far]

        return candidates

    # ----------------------------
    # Frontier detection
    # ----------------------------
    def find_frontier_points(self, grid: OccupancyGrid) -> List[Tuple[float, float]]:
        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        data = grid.data

        def idx(x, y):
            return y * w + x

        pts = []
        stride = max(1, self.stride)
        for y in range(1, h - 1, stride):
            for x in range(1, w - 1, stride):
                if data[idx(x, y)] != -1:
                    continue
                if (
                    data[idx(x + 1, y)] == 0
                    or data[idx(x - 1, y)] == 0
                    or data[idx(x, y + 1)] == 0
                    or data[idx(x, y - 1)] == 0
                ):
                    pts.append((ox + (x + 0.5) * res, oy + (y + 0.5) * res))
        return pts

    def cluster_points(self, pts: List[Tuple[float, float]], radius: float) -> List[List[Tuple[float, float]]]:
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
                pk = pts[k]
                for j in range(len(pts)):
                    if used[j]:
                        continue
                    if d2(pk, pts[j]) <= r2:
                        used[j] = True
                        q.append(j)
                        c.append(pts[j])
            clusters.append(c)
        return clusters

    def centroid(self, cluster: List[Tuple[float, float]]) -> Tuple[float, float]:
        sx = sum(p[0] for p in cluster)
        sy = sum(p[1] for p in cluster)
        n = max(1, len(cluster))
        return (sx / n, sy / n)

    # ----------------------------
    # Claims / Failed with TTL
    # ----------------------------
    def expire_lists(self):
        now = time.time()
        if self.use_claims and self.claim_ttl > 0.0:
            self.claims = [(x, y, t) for (x, y, t) in self.claims if (now - t) <= self.claim_ttl]
        if self.track_failed and self.failed_ttl > 0.0:
            self.failed = [(x, y, t) for (x, y, t) in self.failed if (now - t) <= self.failed_ttl]
        if len(self.failed) > self.max_failed:
            self.failed = self.failed[-self.max_failed:]

    def is_claimed(self, p: Tuple[float, float]) -> bool:
        r2 = self.claim_radius * self.claim_radius
        for (x, y, _) in self.claims:
            if d2(p, (x, y)) <= r2:
                return True
        return False

    def is_failed(self, p: Tuple[float, float]) -> bool:
        r2 = self.failed_radius * self.failed_radius
        for (x, y, _) in self.failed:
            if d2(p, (x, y)) <= r2:
                return True
        return False

    def mark_failed(self, p: Tuple[float, float]):
        if not self.track_failed:
            return
        now = time.time()
        self.failed.append((p[0], p[1], now))
        self.failed = self.failed[-self.max_failed:]

    # ----------------------------
    # k-means (deterministic init)
    # ----------------------------
    def kmeans_deterministic(self, pts: List[Tuple[float, float]], k: int, iters: int):
        centers = [min(pts, key=lambda p: (p[0], p[1]))]
        while len(centers) < k:
            best_p = None
            best_score = -1.0
            for p in pts:
                score = min(d2(p, c) for c in centers)
                if score > best_score:
                    best_score = score
                    best_p = p
            centers.append(best_p)

        labels = [0] * len(pts)
        for _ in range(max(1, iters)):
            for i, p in enumerate(pts):
                labels[i] = min(range(k), key=lambda j: d2(p, centers[j]))
            new_centers = []
            for j in range(k):
                members = [pts[i] for i in range(len(pts)) if labels[i] == j]
                if not members:
                    new_centers.append(centers[j])
                else:
                    sx = sum(m[0] for m in members)
                    sy = sum(m[1] for m in members)
                    new_centers.append((sx / len(members), sy / len(members)))
            centers = new_centers

        return centers, labels

    # ----------------------------
    # TF stability / pose
    # ----------------------------
    def update_merged_tf_stability(self):
        if not self.merged_mode:
            self._merged_tf_is_stable = True
            return
        if self.global_frame is None:
            return

        all_ok = True
        for name in self.robots:
            map_frame = f"{name}/map"
            try:
                _ = self.tf_buffer.lookup_transform(self.global_frame, map_frame, rclpy.time.Time())
            except Exception:
                all_ok = False
                break

        now = time.time()
        if all_ok:
            if self._merged_tf_first_ok_time is None:
                self._merged_tf_first_ok_time = now
            if (now - self._merged_tf_first_ok_time) >= self.merged_tf_stable_sec:
                self._merged_tf_is_stable = True
        else:
            self._merged_tf_first_ok_time = None
            self._merged_tf_is_stable = False

    def get_robot_xy(self) -> Optional[Tuple[float, float]]:
        if self.global_frame is None:
            return None

        candidates = [
            self.base_frame,
            f"{self.robot}/base_footprint",
            f"{self.robot}/base_link",
            "base_footprint",
            "base_link",
        ]

        for base in candidates:
            try:
                t = self.tf_buffer.lookup_transform(self.global_frame, base, rclpy.time.Time())
                return (t.transform.translation.x, t.transform.translation.y)
            except Exception:
                continue
        return None

    # ----------------------------
    # Stop logic
    # ----------------------------
    def _map_change_ratio(self, grid: OccupancyGrid) -> Optional[float]:
        """Compute sampled ratio of changed cells vs previous sample."""
        data = grid.data
        if not data:
            return None

        step = max(1, int(self.map_change_sample_step))
        sample = [data[i] for i in range(0, len(data), step)]

        if self._prev_map_sample is None:
            self._prev_map_sample = sample
            return None

        prev = self._prev_map_sample
        n = min(len(prev), len(sample))
        if n <= 0:
            self._prev_map_sample = sample
            return None

        changed = 0
        for i in range(n):
            if sample[i] != prev[i]:
                changed += 1

        self._prev_map_sample = sample
        return changed / float(n)

    def _update_map_stability(self):
        if self.map_msg is None:
            return

        ratio = self._map_change_ratio(self.map_msg)
        now = time.time()
        if ratio is None:
            return

        if ratio <= self.map_change_ratio_threshold:
            if self._map_stable_first_time is None:
                self._map_stable_first_time = now
        else:
            self._map_stable_first_time = None

        # occasional log (helps debugging)
        if now - self._last_map_stable_log > 5.0 and self.stop_when_merge_satisfied and self.merged_mode:
            stable_for = 0.0 if self._map_stable_first_time is None else (now - self._map_stable_first_time)
            self.get_logger().info(
                f"   merge-check: map_change_ratio={ratio:.5f}, map_stable_for={stable_for:.1f}s, tf_stable={self._merged_tf_is_stable}"
            )
            self._last_map_stable_log = now

    def maybe_stop(self):
        # Always update map-stability tracker (only used if stop_when_merge_satisfied=True)
        self._update_map_stability()

        # 1) Classic stop: no frontiers for N ticks + TF stable if required
        if self.no_frontier_ticks < self.no_frontier_ticks_to_stop:
            return

        tf_ok = (not self.require_merged_tf_to_stop) or self._merged_tf_is_stable
        if not tf_ok:
            now = time.time()
            if now - self._last_merge_warn > 3.0:
                self.get_logger().warn("Frontiers empty but merged TF not stable yet -> NOT stopping.")
                self._last_merge_warn = now
            return

        # 2) If user wants "stop only when merge is satisfied", apply stronger conditions
        if self.stop_when_merge_satisfied and self.merged_mode:
            now = time.time()
            runtime = now - self.start_time
            if runtime < self.merge_min_runtime_sec:
                return

            if self._map_stable_first_time is None:
                return

            if (now - self._map_stable_first_time) < self.map_stable_sec:
                return

            # Passed all stronger conditions
            self.finish(reason="merge_satisfied")
            return

        # Otherwise stop with classic condition
        self.finish(reason="no_frontiers")

    def finish(self, reason="done"):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"✅ Exploration complete (reason={reason})")
        self.get_logger().info(f"⏱️ Total exploration time: {elapsed:.1f} sec")
        try:
            self.timer.cancel()
        except Exception:
            pass

    # ----------------------------
    # Nav2 goal send (with optional frame transform)
    # ----------------------------
    def _transform_xy(self, xy: Tuple[float, float], src_frame: str, dst_frame: str) -> Optional[Tuple[float, float]]:
        """Transform a point (x,y) from src_frame -> dst_frame using TF at latest time."""
        if src_frame == dst_frame:
            return xy
        try:
            tf = self.tf_buffer.lookup_transform(dst_frame, src_frame, rclpy.time.Time())
        except Exception:
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        rx, ry = rotate2d(xy[0], xy[1], yaw)
        return (rx + tx, ry + ty)

    def send_goal(self, xy_global: Tuple[float, float]):
        if self.global_frame is None or self.nav_goal_frame is None:
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            now = time.time()
            if now - self._last_nav_server_warn > 2.0:
                self.get_logger().warn(f"Nav2 action server not available: {self.nav_action}")
                self._last_nav_server_warn = now
            return

        # Convert goal from global_frame into nav_goal_frame if needed
        xy_nav = self._transform_xy(xy_global, self.global_frame, self.nav_goal_frame)
        if xy_nav is None:
            now = time.time()
            if now - self._last_nav_frame_warn > 2.0:
                self.get_logger().warn(
                    f"Cannot TF transform goal {self.global_frame} -> {self.nav_goal_frame}. Skipping goal."
                )
                self._last_nav_frame_warn = now
            return

        # publish claim in GLOBAL frame (shared)
        if self.use_claims and self.claim_pub is not None:
            claim = PoseStamped()
            claim.header.frame_id = self.global_frame
            claim.header.stamp = self.get_clock().now().to_msg()
            claim.pose.position.x = float(xy_global[0])
            claim.pose.position.y = float(xy_global[1])
            claim.pose.orientation.w = 1.0
            self.claim_pub.publish(claim)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.nav_goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy_nav[0])
        goal.pose.pose.position.y = float(xy_nav[1])
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"➡️ {self.robot} → ({xy_global[0]:.2f}, {xy_global[1]:.2f}) global={self.global_frame} nav_goal_frame={self.nav_goal_frame}"
        )

        self.goal_active = True
        self.goal_sent_time = time.time()
        self.goal_xy_sent = xy_global

        self.last_goal = xy_global
        self.last_goal_time = time.time()

        self.nav_client.send_goal_async(goal).add_done_callback(self._resp)

    def _resp(self, future):
        try:
            h = future.result()
        except Exception as e:
            self.get_logger().warn(f"Goal send exception: {e}")
            self.goal_active = False
            if self.goal_xy_sent:
                self.mark_failed(self.goal_xy_sent)
            return

        if not h.accepted:
            self.get_logger().warn("Goal rejected")
            self.goal_active = False
            if self.goal_xy_sent:
                self.mark_failed(self.goal_xy_sent)
            return

        h.get_result_async().add_done_callback(self._done)

    def _done(self, future):
        try:
            res = future.result()
            status = int(res.status)
            err = int(res.result.error_code)
            self.get_logger().info(f"🏁 Goal finished: status={status} nav2_error_code={err}")

            # 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
            if status != 4:
                if self.goal_xy_sent:
                    self.mark_failed(self.goal_xy_sent)

        except Exception as e:
            self.get_logger().warn(f"Goal result exception: {e}")
            if self.goal_xy_sent:
                self.mark_failed(self.goal_xy_sent)

        self.goal_active = False
        self.goal_xy_sent = None

    def check_timeout(self):
        if time.time() - self.goal_sent_time > self.goal_timeout:
            self.get_logger().warn("⏱️ Goal timeout -> marking failed + releasing goal")
            if self.goal_xy_sent:
                self.mark_failed(self.goal_xy_sent)
            self.goal_active = False
            self.goal_xy_sent = None


def main():
    rclpy.init()
    node = ExplorerMyIdea()
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


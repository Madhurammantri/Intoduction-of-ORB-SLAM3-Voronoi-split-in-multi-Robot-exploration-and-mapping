#!/usr/bin/env python3
"""
auto_explore_merged_idea_A.py

Primary author
--------------
Mr. Divid (Robotics Professor, TU Budapest)

Summary
-------
IDEA A: Voronoi split on a MERGED occupancy grid (/map):
- Each robot computes frontiers from the same merged map
- Robots avoid duplicates using /claimed_goals
- Region split via Voronoi partition based on peer robot poses

STOP LOGIC (important):
- We DO NOT stop just because frontiers are empty.
- We stop only after merged-map TF is stable (global_frame -> robot_i/map exists for all peers)
  AND frontiers are empty for N ticks.

Attribution / Inspiration
-------------------------
This program uses the standard "frontier-based exploration" concept.
The *basic frontier definition* in `find_frontiers()` (unknown cell adjacent to free cell)
is inspired by a minimal frontier explorer reference script (e.g., `explore_map.py` / `explore_map2.py`).

All other multi-robot coordination logic in this file (merged-map operation, goal claiming,
Voronoi assignment based on peer poses, TF-based peer pose updates, TF-stability-gated stopping,
clustering and selection policy, Nav2 integration details) is an original implementation by the
primary author named above.

IMPORTANT in multi-robot namespace setups:
- If TF is namespaced (e.g. /robot_1/tf), you MUST remap when running:
    -r /tf:=/robot_1/tf -r /tf_static:=/robot_1/tf_static
(But if your launch relays /robot_i/tf -> /tf, no remap needed.)
"""

import math
import time
from typing import List, Tuple, Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros


def d2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


# =============================================================================
# IMPROVEMENT: Peer list parsing to support multi-robot Voronoi partitioning
# (used to define the "seed" positions per robot; later updated live from TF)
# =============================================================================
def parse_peers(peers_str: str) -> List[Tuple[str, float, float]]:
    """
    peers format: "robot_1:0,0;robot_2:5,0"
    returns: [(name,x,y), ...]
    """
    peers: List[Tuple[str, float, float]] = []
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


class ExplorerIdeaA(Node):
    def __init__(self):
        super().__init__("auto_explore_merged_idea_A")

        # --------- Params ----------
        self.declare_parameter("robot_name", "robot_1")

        # =============================================================================
        # IMPROVEMENT: Explore on MERGED map topic (single shared /map for all robots)
        # =============================================================================
        self.declare_parameter("map_topic", "/map")

        # nav2 action for THIS robot
        self.declare_parameter("nav_action", "")  # default computed

        # TF frames
        # Use "auto" to auto-detect from map.header.frame_id
        self.declare_parameter("global_frame", "auto")
        self.declare_parameter("base_frame", "")  # default computed: {robot}/base_link

        # frontier detection
        self.declare_parameter("stride", 2)                # was 4 (too sparse on thin frontiers)
        self.declare_parameter("cluster_radius", 0.8)      # meters
        self.declare_parameter("min_cluster_size", 6)      # was 8 (can be too strict)

        # =============================================================================
        # IMPROVEMENT: Multi-robot coordination via GOAL CLAIMING
        # (prevents both robots from navigating to the same frontier/cluster)
        # =============================================================================
        self.declare_parameter("tick_sec", 2.0)
        self.declare_parameter("claim_topic", "/claimed_goals")
        self.declare_parameter("claim_radius", 1.2)        # meters
        self.declare_parameter("min_goal_separation", 0.8)
        self.declare_parameter("goal_timeout_sec", 180.0)

        # =============================================================================
        # IMPROVEMENT: Voronoi region split (robot "owns" frontiers closest to its pose)
        # =============================================================================
        self.declare_parameter("peers", "robot_1:0,0;robot_2:5,0")

        # =============================================================================
        # IMPROVEMENT: Robust STOP gating based on merged TF stability (post-merge)
        # =============================================================================
        self.declare_parameter("no_frontier_ticks_to_stop", 15)      # was hardcoded 10
        self.declare_parameter("require_merged_tf_to_stop", True)    # gate stopping on merge TF
        self.declare_parameter("merged_tf_stable_sec", 8.0)          # TF must be valid for this long

        # --------- Read params ----------
        self.robot = self.get_parameter("robot_name").value
        self.map_topic = self.get_parameter("map_topic").value

        nav_action = self.get_parameter("nav_action").value
        self.nav_action = nav_action if nav_action else f"/{self.robot}/navigate_to_pose"

        self.global_frame_param = self.get_parameter("global_frame").value  # may be "auto"
        self.global_frame: Optional[str] = None  # resolved later (after map arrives)

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
        self.peer_names = [p[0] for p in self.peers]

        # stop params
        self.no_frontier_ticks_to_stop = int(self.get_parameter("no_frontier_ticks_to_stop").value)
        self.require_merged_tf_to_stop = bool(self.get_parameter("require_merged_tf_to_stop").value)
        self.merged_tf_stable_sec = float(self.get_parameter("merged_tf_stable_sec").value)

        # --------- State ----------
        self.map_msg: Optional[OccupancyGrid] = None
        self.claims: List[Tuple[float, float]] = []
        self.last_goal: Optional[Tuple[float, float]] = None

        self.goal_active = False
        self.goal_sent_time = 0.0

        self.no_frontier_ticks = 0

        self.start_time = time.time()
        self.start_sim_time = self.get_clock().now()


        # merged TF gating state (NEW)
        self._merged_tf_first_ok_time: Optional[float] = None
        self._merged_tf_is_stable: bool = False
        self._logged_peer_seed_once: bool = False

        # Throttle logs
        self._last_tf_warn = 0.0
        self._last_nav_warn = 0.0
        self._last_merge_warn = 0.0

        # --------- TF ----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --------- ROS Interfaces ----------
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, map_qos)

        # =============================================================================
        # IMPROVEMENT: Claim pub/sub channel used to avoid duplicate exploration goals
        # =============================================================================
        claim_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        self.claim_pub = self.create_publisher(PoseStamped, self.claim_topic, claim_qos)
        self.create_subscription(PoseStamped, self.claim_topic, self.claim_cb, claim_qos)

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)

        self.timer = self.create_timer(self.tick_sec, self.tick)

        self.get_logger().info(f"🚀 Exploration started for {self.robot}")
        self.get_logger().info(f"   map_topic: {self.map_topic}")
        self.get_logger().info(f"   nav_action: {self.nav_action}")
        self.get_logger().info(f"   global_frame(param): {self.global_frame_param}")
        self.get_logger().info(f"   base_frame: {self.base_frame}")

    # ---------- Callbacks ----------
    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

        # =============================================================================
        # IMPROVEMENT: Auto-resolve global frame from map header (reduces TF mismatch pain)
        # =============================================================================
        if self.global_frame is None:
            if (self.global_frame_param or "").strip().lower() == "auto":
                self.global_frame = (msg.header.frame_id or "").strip() or "map"
            else:
                self.global_frame = self.global_frame_param

            self.get_logger().info(
                f"✅ Resolved global_frame = {self.global_frame} (map header.frame_id={msg.header.frame_id})"
            )

    def claim_cb(self, msg: PoseStamped):
        # Only accept claims in our global frame (once resolved)
        if self.global_frame is None:
            return
        if msg.header.frame_id != self.global_frame:
            return
        self.claims.append((msg.pose.position.x, msg.pose.position.y))
        if len(self.claims) > 200:
            self.claims = self.claims[-200:]

    # ---------- Core loop ----------
    def tick(self):
        if self.goal_active:
            self.check_timeout()
            return

        if self.map_msg is None or self.global_frame is None:
            self.get_logger().warn("Waiting for merged map...")
            return

        # =============================================================================
        # IMPROVEMENT: Update peer seeds LIVE from TF (dynamic Voronoi partition)
        # =============================================================================
        self.update_peer_seeds_from_tf()

        # =============================================================================
        # IMPROVEMENT: Track merged-map TF stability (used ONLY to decide when it is safe to stop)
        # =============================================================================
        self.update_merged_tf_stability()

        # Don’t block here — just check if nav2 action server is up
        if not self.nav_client.wait_for_server(timeout_sec=0.0):
            now = time.time()
            if now - self._last_nav_warn > 2.0:
                self.get_logger().warn("Nav2 not ready (navigate_to_pose action server not available yet)")
                self._last_nav_warn = now
            return

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            now = time.time()
            if now - self._last_tf_warn > 2.0:
                self.get_logger().warn("Waiting for TF...")
                self._last_tf_warn = now
            return

        # =============================================================================
        # INSPIRED SEGMENT (concept): Frontier extraction from occupancy grid
        # - Using the standard idea: UNKNOWN (-1) adjacent to FREE (0) is a frontier
        # - See `find_frontiers()` for the exact definition
        # =============================================================================
        frontiers = self.find_frontiers(self.map_msg)

        # =============================================================================
        # IMPROVEMENT: Cluster frontiers into stable navigation targets (reduces noise/micro-goals)
        # =============================================================================
        clusters = self.cluster_frontiers(frontiers)

        # =============================================================================
        # IMPROVEMENT: Pick a goal that is:
        # - owned by this robot (Voronoi), and
        # - not too close to any claimed goal (claiming), and
        # - not too close to the last goal (min separation)
        # =============================================================================
        goal = self.pick_owned_goal(clusters, robot_xy)
        if goal is None:
            self.no_frontier_ticks += 1

            # =============================================================================
            # IMPROVEMENT: STOP only after merge TF is stable (if enabled) AND frontiers empty long enough
            # - Prevents premature stop before map frames are properly merged/consistent
            # =============================================================================
            if self.no_frontier_ticks >= self.no_frontier_ticks_to_stop:
                if (not self.require_merged_tf_to_stop) or self._merged_tf_is_stable:
                    self.get_logger().info("✅ No valid frontiers left (after merge). Exploration done.")
                    elapsed = time.time() - self.start_time
                    elapsed_sim = (self.get_clock().now() - self.start_sim_time).nanoseconds / 1e9
                    self.get_logger().info(f"⏱️ Total exploration time: {elapsed:.1f} sec")
                    self.get_logger().info(f"Exploration time: real={elapsed:.2f}s sim={elapsed_sim:.2f}s")
                    self.timer.cancel()
                else:
                    now = time.time()
                    if now - self._last_merge_warn > 3.0:
                        self.get_logger().warn(
                            "Frontiers empty, but merged TF not stable yet → NOT stopping (waiting for proper merge)"
                        )
                        self._last_merge_warn = now
            return

        self.no_frontier_ticks = 0
        self.send_goal(goal)

    # ---------- Merge TF gating (NEW) ----------
    # =============================================================================
    # IMPROVEMENT: Use TF to update peer positions (Voronoi seeds) in the global frame
    # =============================================================================
    def update_peer_seeds_from_tf(self):
        """Update self.peers (x,y) from TF: global_frame -> peer/base_link."""
        if self.global_frame is None:
            return

        updated: List[Tuple[str, float, float]] = []
        any_ok = False

        for name in self.peer_names:
            base = f"{name}/base_link"
            try:
                t = self.tf_buffer.lookup_transform(self.global_frame, base, rclpy.time.Time())
                x = float(t.transform.translation.x)
                y = float(t.transform.translation.y)
                updated.append((name, x, y))
                any_ok = True
            except Exception:
                # keep old seed if TF not available
                old = next((p for p in self.peers if p[0] == name), None)
                if old is not None:
                    updated.append(old)
                else:
                    updated.append((name, 0.0, 0.0))

        self.peers = updated

        if any_ok and not self._logged_peer_seed_once:
            self.get_logger().info("✅ Peer seeds updated from TF (used for Voronoi split):")
            for name, x, y in self.peers:
                self.get_logger().info(f"   {name}: ({x:.2f}, {y:.2f})")
            self._logged_peer_seed_once = True

    # =============================================================================
    # IMPROVEMENT: Stability detection for merge TF (global_frame -> robot_i/map for all peers)
    # - Only used to decide when it is safe to stop exploration
    # =============================================================================
    def update_merged_tf_stability(self):
        """
        Consider merge 'stable' if TF exists from global_frame -> robot_i/map for ALL peers
        continuously for merged_tf_stable_sec.
        """
        if self.global_frame is None:
            return

        all_ok = True
        for name in self.peer_names:
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

    # ---------- TF ----------
    def get_robot_xy(self) -> Optional[Tuple[float, float]]:
        assert self.global_frame is not None

        # Try robot/base_link, fallback to base_link
        for base in [self.base_frame, "base_link"]:
            try:
                t = self.tf_buffer.lookup_transform(self.global_frame, base, rclpy.time.Time())
                return (t.transform.translation.x, t.transform.translation.y)
            except Exception:
                continue
        return None

    # ---------- Frontier detection ----------
    # =============================================================================
    # INSPIRED SEGMENT (implementation concept): Basic frontier definition
    # - Frontier = UNKNOWN cell (-1) that is adjacent (4-neighborhood) to a FREE cell (0)
    # - This is a standard approach and is inspired by a minimal frontier explorer reference
    # =============================================================================
    def find_frontiers(self, grid: OccupancyGrid) -> List[Tuple[float, float]]:
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        data = grid.data

        def idx(x, y):
            return y * w + x

        frontiers: List[Tuple[float, float]] = []
        for y in range(1, h - 1, self.stride):
            for x in range(1, w - 1, self.stride):
                if data[idx(x, y)] != -1:
                    continue
                # frontier = unknown cell adjacent to free
                if any(data[idx(nx, ny)] == 0 for nx, ny in [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]):
                    wx = ox + (x + 0.5) * res
                    wy = oy + (y + 0.5) * res
                    frontiers.append((wx, wy))
        return frontiers

    # =============================================================================
    # IMPROVEMENT: Frontier clustering into centers (stabilizes goals, reduces duplicates/noise)
    # =============================================================================
    def cluster_frontiers(self, pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        # super simple radius clustering (good enough for now)
        clusters: List[List[Tuple[float, float]]] = []
        r2 = self.cluster_radius ** 2

        for p in pts:
            placed = False
            for c in clusters:
                if d2(p, c[0]) <= r2:
                    c.append(p)
                    placed = True
                    break
            if not placed:
                clusters.append([p])

        centers: List[Tuple[float, float]] = []
        for c in clusters:
            if len(c) < self.min_cluster_size:
                continue
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            centers.append((cx, cy))
        return centers

    # ---------- Region split (Voronoi on peer poses) ----------
    # =============================================================================
    # IMPROVEMENT: Voronoi ownership (assign each candidate to nearest peer pose)
    # =============================================================================
    def owner_of(self, pt: Tuple[float, float]) -> str:
        best = None
        for name, x, y in self.peers:
            dist = d2(pt, (x, y))
            if best is None or dist < best[0]:
                best = (dist, name)
        return best[1] if best else self.robot

    # =============================================================================
    # IMPROVEMENT: Candidate filtering combines Voronoi ownership + claim avoidance + separation
    # =============================================================================
    def pick_owned_goal(
        self,
        clusters: List[Tuple[float, float]],
        robot_xy: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        candidates: List[Tuple[float, Tuple[float, float]]] = []
        for c in clusters:
            if self.owner_of(c) != self.robot:
                continue
            if self.last_goal and d2(c, self.last_goal) < self.min_goal_sep ** 2:
                continue
            if any(d2(c, g) < self.claim_radius ** 2 for g in self.claims):
                continue
            # pick nearest goal to current robot pose
            candidates.append((d2(c, robot_xy), c))

        if not candidates:
            return None

        candidates.sort(key=lambda x: x[0])
        return candidates[0][1]

    # ---------- Nav ----------
    def send_goal(self, xy: Tuple[float, float]):
        assert self.global_frame is not None

        # =============================================================================
        # IMPROVEMENT: Publish a claim BEFORE sending the Nav2 goal (deconflicts multi-robot runs)
        # =============================================================================
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

        self.nav_client.send_goal_async(goal).add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"Goal send failed: {e}")
            self.goal_active = False
            return

        if not handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.goal_active = False
            return

        handle.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, future):
        self.get_logger().info("🏁 Goal finished")
        self.goal_active = False

    def check_timeout(self):
        if time.time() - self.goal_sent_time > self.goal_timeout:
            self.get_logger().warn("⏱️ Goal timeout, retrying")
            self.goal_active = False


def main():
    rclpy.init()
    node = ExplorerIdeaA()
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


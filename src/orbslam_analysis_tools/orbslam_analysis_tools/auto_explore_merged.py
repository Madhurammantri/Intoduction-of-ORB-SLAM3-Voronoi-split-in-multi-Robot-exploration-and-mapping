#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
import time
import math
import json
import random


class AutoExploreMerged(Node):

    def __init__(self):
        super().__init__('auto_explore_merged')

        # -------- Parameters --------
        self.robot_name = self.declare_parameter(
            'robot_name', 'robot_1').value

        self.map_topic = f'/{self.robot_name}/map'
        self.nav_action = f'/{self.robot_name}/navigate_to_pose'
        self.claim_topic = '/claimed_goals'

        self.get_logger().info(f'🚀 Auto exploration (merged) started for {self.robot_name}')
        self.get_logger().info(f'   map_topic: {self.map_topic}')
        self.get_logger().info(f'   nav_action: {self.nav_action}')
        self.get_logger().info(f'   claim_topic: {self.claim_topic}')

        # -------- State --------
        self.map = None
        self.claimed = set()
        self.current_goal = None
        self.exploring = False
        self.start_time = time.time()
        self.no_frontier_count = 0

        # -------- ROS Interfaces --------
        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        self.create_subscription(
            String,
            self.claim_topic,
            self.claim_callback,
            10
        )

        self.claim_pub = self.create_publisher(
            String,
            self.claim_topic,
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.nav_action
        )

        self.timer = self.create_timer(2.0, self.loop)

    # --------------------------------------------------

    def map_callback(self, msg):
        self.map = msg

    def claim_callback(self, msg):
        try:
            data = json.loads(msg.data)
            key = (data['gx'], data['gy'])
            self.claimed.add(key)
        except Exception:
            pass

    # --------------------------------------------------

    def loop(self):
        if self.map is None:
            self.get_logger().warn('Waiting for map...')
            return

        if self.exploring:
            return

        frontier = self.find_frontier()
        if frontier is None:
            self.no_frontier_count += 1
            if self.no_frontier_count > 10:
                total = time.time() - self.start_time
                self.get_logger().info(f'✅ Exploration complete')
                self.get_logger().info(f'⏱️ Total exploration time: {total:.1f} sec')
                self.destroy_node()
            return

        self.no_frontier_count = 0
        gx, gy = frontier
        self.claim_goal(gx, gy)
        self.send_goal(gx, gy)

    # --------------------------------------------------

    def find_frontier(self):
        w = self.map.info.width
        h = self.map.info.height
        data = self.map.data

        candidates = []

        for y in range(1, h - 1):
            for x in range(1, w - 1):
                idx = y * w + x
                if data[idx] != 0:
                    continue

                neighbors = [
                    data[(y + 1) * w + x],
                    data[(y - 1) * w + x],
                    data[y * w + (x + 1)],
                    data[y * w + (x - 1)],
                ]

                if -1 in neighbors:
                    if (x, y) not in self.claimed:
                        candidates.append((x, y))

        if not candidates:
            return None

        return random.choice(candidates)

    # --------------------------------------------------

    def claim_goal(self, gx, gy):
        msg = String()
        msg.data = json.dumps({
            'robot': self.robot_name,
            'gx': gx,
            'gy': gy
        })
        self.claim_pub.publish(msg)

    # --------------------------------------------------

    def send_goal(self, gx, gy):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y

        x = ox + gx * res
        y = oy + gy * res

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = f'{self.robot_name}/map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'➡️ {self.robot_name} → ({x:.2f}, {y:.2f})')

        self.exploring = True
        send = self.nav_client.send_goal_async(goal)
        send.add_done_callback(self.goal_response)

    # --------------------------------------------------

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.exploring = False
            return

        result = goal_handle.get_result_async()
        result.add_done_callback(self.goal_done)

    def goal_done(self, future):
        self.exploring = False
        self.get_logger().info('🏁 Goal finished')


# ======================================================

def main():
    rclpy.init()
    node = AutoExploreMerged()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


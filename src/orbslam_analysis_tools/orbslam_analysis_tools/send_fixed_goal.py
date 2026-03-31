#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


class FixedGoalSender(Node):
    def __init__(self):
        super().__init__('fixed_goal_sender')

        self.client = ActionClient(
            self,
            NavigateToPose,
            '/robot_1/navigate_to_pose'
        )

        self.get_logger().info("Waiting for NavigateToPose action server...")
        self.client.wait_for_server()

        self.send_goal()

    def send_goal(self):
        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'robot_1/map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # 🔒 FIXED GOAL (CHANGE ONCE, NEVER AGAIN)
        pose.pose.position.x = -4.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, 1.57)  # 90 degrees
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal.pose = pose

        self.get_logger().info("Sending fixed goal to robot_1")
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = FixedGoalSender()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


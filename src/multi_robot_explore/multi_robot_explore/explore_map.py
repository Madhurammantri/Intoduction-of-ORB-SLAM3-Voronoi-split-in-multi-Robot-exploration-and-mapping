import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener
import numpy as np
import rclpy.time


class MultiRobotExplorer(Node):

    def __init__(self):
        super().__init__('multi_robot_explorer')

        # Parameters
        self.declare_parameter('min_unknown_cells', 15)
        self.min_unknown_cells = self.get_parameter('min_unknown_cells').value

        # Clock
        self.latest_clock = None
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map subscriptions
        self.local_map_1 = None
        self.local_map_2 = None
        self.map1_ready = False
        self.map2_ready = False

        self.create_subscription(
            OccupancyGrid, '/robot_1/map', self.robot1_map_callback, 10
        )
        self.create_subscription(
            OccupancyGrid, '/robot_2/map', self.robot2_map_callback, 10
        )

        # Goal publishers (Nav2-compatible topic)
        self.pub_1 = self.create_publisher(
            PoseStamped, '/robot_1/goal_pose', 10
        )
        self.pub_2 = self.create_publisher(
            PoseStamped, '/robot_2/goal_pose', 10
        )

        # Timer
        self.timer = self.create_timer(2.0, self.explore)

    def clock_callback(self, msg):
        self.latest_clock = msg.clock

    def robot1_map_callback(self, msg):
        self.local_map_1 = msg
        self.map1_ready = True

    def robot2_map_callback(self, msg):
        self.local_map_2 = msg
        self.map2_ready = True

    def explore(self):
        if not self.map1_ready or not self.map2_ready:
            self.get_logger().warn("Waiting for all maps...")
            return

        frontiers_1 = self.find_frontiers(self.local_map_1)
        frontiers_2 = self.find_frontiers(self.local_map_2)

        if frontiers_1:
            self.send_goal(frontiers_1[0], self.local_map_1, 'robot_1/map', self.pub_1)

        if frontiers_2:
            self.send_goal(frontiers_2[0], self.local_map_2, 'robot_2/map', self.pub_2)

    def send_goal(self, cell, map_msg, frame, pub):
        y, x = cell
        res = map_msg.info.resolution
        origin = map_msg.info.origin.position

        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.header.stamp = (
            self.latest_clock if self.latest_clock
            else self.get_clock().now().to_msg()
        )

        goal.pose.position.x = origin.x + (x + 0.5) * res
        goal.pose.position.y = origin.y + (y + 0.5) * res
        goal.pose.orientation.w = 1.0

        pub.publish(goal)

    def find_frontiers(self, map_msg):
        h, w = map_msg.info.height, map_msg.info.width
        data = np.array(map_msg.data).reshape(h, w)

        frontiers = []
        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if data[y, x] != 0:
                    continue
                if -1 in data[y-1:y+2, x-1:x+2]:
                    frontiers.append((y, x))
        return frontiers


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


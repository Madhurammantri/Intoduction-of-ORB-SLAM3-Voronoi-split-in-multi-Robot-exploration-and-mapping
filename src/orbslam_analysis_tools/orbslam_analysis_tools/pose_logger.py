import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import csv
import os


class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        self.declare_parameter(
            'output_file',
            '/home/madhuram/Desktop/multi_robot_orb_ws/pose_logs/robot_1_pose_log.csv'
        )
        self.output_file = (
            self.get_parameter('output_file')
            .get_parameter_value()
            .string_value
        )

        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        self.file = open(self.output_file, 'w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            'time_sec',
            'source',
            'x', 'y', 'z',
            'qx', 'qy', 'qz', 'qw'
        ])

        self.create_subscription(
            Odometry,
            '/robot_1/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/robot_1/slam/pose',
            self.slam_callback,
            10
        )

        self.get_logger().info(f'Logging poses to {self.output_file}')

    def odom_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        self.writer.writerow([
            t, 'odom',
            p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w
        ])

    def slam_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose
        self.writer.writerow([
            t, 'orb_slam',
            p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w
        ])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PoseLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


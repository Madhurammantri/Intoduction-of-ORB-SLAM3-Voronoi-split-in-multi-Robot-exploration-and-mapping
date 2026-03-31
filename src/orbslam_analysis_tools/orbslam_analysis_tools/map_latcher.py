import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class MapLatcher(Node):
    def __init__(self):
        super().__init__('map_latcher')

        self.declare_parameter('robot_name', 'robot_1')
        robot = self.get_parameter('robot_name').value

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.sub = self.create_subscription(
            OccupancyGrid,
            f'/{robot}/map',
            self.map_callback,
            10
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            f'/{robot}/map_latched',
            qos
        )

        self.get_logger().info(f'📌 Map latcher running for {robot}')

    def map_callback(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MapLatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from simple_commander import SimpleCommander

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.commander = SimpleCommander(self)

        # Use LiDAR to detect a close object -- may use the standard camera instead
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        is_obstacle_detected = min(msg.ranges) < 0.5

        if is_obstacle_detected:
            # Stop the vehicle
            self.commander.stop()

            # Obstable avoidance behavior using Simple Commander and a state machine
            self.commander.rotate(90)

    def stop(self):
        # Stop the vehicle when the node is stopped
        self.commander.stop()
        super().stop()


def main(args=None):
    rclpy.init(args=args)

    obstacle_avoidance_node = ObstacleAvoidanceNode()

    rclpy.spin(obstacle_avoidance_node)

    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# potentially used to encapsulate Simple Commander and implement high-level patrol vehicle behaviors
# Simpe Commander can go-to-pose, but we plan to use this node for higher-level functions, including
# goToStation(), goToBase(), goAroundObstacle(), etc.
#
# 
import rclpy
from rclpy.node import Node
from simple_commander import Commander

class Nav2Node(Node):
    def __init__(self):
        super().__init__('nav2_node')
        self._commander = Commander(self)

    def send_goal(self, pose):
        self._commander.go_to_pose(pose)

def main(args=None):
    rclpy.init(args=args)
    nav2_node = Nav2Node()
    rclpy.spin(nav2_node)
    nav2_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
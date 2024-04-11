# potentially used to encapsulate Simple Commander and implement high-level patrol vehicle behaviors
# SC can go-to-pose, but we might want to put the go-back-to-base logic here so it can be called by the patrol node
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
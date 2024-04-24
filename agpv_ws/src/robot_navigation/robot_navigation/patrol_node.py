# This is the main controller node
# See the state diagram for a detailed behavior
# it runs a routine patrol unless pre-empted by an obstacle or the need to obey a traffic signal
# patrolling involves checking the health of the stations visited, logging the visit, and notifying if the station has an issue
# at the end of the patrol it should return to base
# It uses nav2_node to navigate to stations and return to base
import rclpy
from rclpy.node import Node

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

    def patrol(self, waypoints):
        # Patrol behavior
        # defined using a Moore-style state machine
        # if state == "Patrol" perform routine patrol, otherwise yield control to other state machines
        # including obstacle avoidance and traffic light detection behaviors
        while (station left to be visisted and state == "patrolling"):
            # Navigate to station if no red traffic light (wait until traffic light not red)
            self.get_logger().info(f'Navigating to waypoint {waypoint}')

            # Check station health 
            self.get_logger().info('Checking station health')

            # Log visit
            self.get_logger().info('Logging visit')

            # Report unhealthy station
            self.get_logger().info('Reporting unhealthy station')

        # After all waypoints have been visited, return to base
        self.get_logger().info('Returning to base')
        # Use simple-commander to return to base

def main(args=None):
    rclpy.init(args=args)
    patrol_node = PatrolNode()
    rclpy.spin(patrol_node)
    patrol_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
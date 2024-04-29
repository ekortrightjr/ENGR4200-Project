#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, String
import time
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import math
# from rcl_interfaces.msg import Parameter, ParameterValue
# from rcl_interfaces.srv import SetParameters

class PatrolVehicle(Node):
    def __init__(self):
        super().__init__('patrol_vehicle');
        self.subscription = self.create_subscription(
            String,
            '/red_light',
            self.listener_callback,
            10)
        self.state = "Idle"

        print('__init__ complete')

    def listener_callback(self, msg):
        print(msg.data)
        if msg.data == "Red light is on":
          if self.state == "Idle":
            self.state = "Ready"
        else:
          if self.state == "Ready":
            self.state = "Patrol"
            self.patrol()
        print(self.state)

    def patrol(self):
      print("patrol()")
      nav = BasicNavigator()

      # create initial pose
      initial_pose = get_pose_stampted(nav=nav, p={"x": -0.0321, "y": -0.0403}, a=0.0490)
      nav.setInitialPose(initial_pose)
      
      # wait for mav2
      nav.waitUntilNav2Active()
      time.sleep(5)

      # Go to Station 1
      goal_pose = get_pose_stampted(nav=nav, p={"x": 1.6089, "y": -0.4549}, a=0.0494)
      nav.goToPose(goal_pose)

      # wait for task to be completed
      while not nav.isTaskComplete():
          feedback = nav.getFeedback()
          print(feedback)

      print("On to second station!")
      time.sleep(10)

      # Go to Station 2
      goal_pose = get_pose_stampted(nav=nav, p={"x": 1.3664, "y": 0.1324}, a=1.5419)
      nav.goToPose(goal_pose)

      # wait for task to be completed
      while not nav.isTaskComplete():
          feedback = nav.getFeedback()
          print(feedback)

      print("Back to base!")
      time.sleep(10)

      # Go back to base
      nav.goToPose(initial_pose)

      # wait for task to be completed
      while not nav.isTaskComplete():
          feedback = nav.getFeedback()
          print(feedback)

      print(nav.getResult())
      time.sleep(5)
      print("end patrol()")


def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return qx, qy, qz, qw

def get_pose_stampted(nav, p, a):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, a)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = p["x"]
    pose.pose.position.y = p["y"]
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main(args = None):
    print('main')
    rclpy.init(args = args)
    patrol_vehicle = PatrolVehicle()
    rclpy.spin(patrol_vehicle)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
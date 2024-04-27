#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import math
import time

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

def main():
    rclpy.init()
    nav = BasicNavigator()

    # create initial pose
    initial_pose = get_pose_stampted(nav=nav, p={"x": -0.07, "y": -0.04}, a=0.3)
    nav.setInitialPose(initial_pose)
    
    # wait for mav2
    nav.waitUntilNav2Active()
    time.sleep(5)

    # # send a nav2 goal
    goal_pose = get_pose_stampted(nav=nav, p={"x": 1.12, "y": 0.55}, a=0.65)
    nav.goToPose(goal_pose)

    # wait for task to be completed
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    print("On to second station!")
    time.sleep(10)
    # # send a nav2 goal
    goal_pose = get_pose_stampted(nav=nav, p={"x": 0.93, "y": -0.09}, a=1.57)
    nav.goToPose(goal_pose)

    # wait for task to be completed
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    waypoints = [
#        get_pose_stampted(nav, {"x": 3.5, "y": 2.0}, 3.14),
#        get_pose_stampted(nav, {"x": 0.0, "y": 2.0}, 4.71),
#        get_pose_stampted(nav, {"x": 0.0, "y": 0.0}, 0.0)
        get_pose_stampted(nav=nav, p={"x": -0.2, "y": 1.33}, a=2.9),
        get_pose_stampted(nav=nav, p={"x": -0.79, "y": -0.94}, a=2.8)
    ]

    #nav.followWaypoints(waypoints)

    # wait for task to be completed
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    print(nav.getResult())

    rclpy.shutdown()

if __name__ == '__main__':
    main()

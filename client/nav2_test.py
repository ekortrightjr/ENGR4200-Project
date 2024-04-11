#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def get_pose_stampted(nav, p, a):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, a)
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
    initial_pose = get_pose_stampted(nav=nav, p={"x": 0.0, "y": 0.0}, a=0.0)
    nav.setInitialPose(initial_pose)
    
    # wait for mav2
    nav.waitUntilNav2Active()
    
    # # send a nav2 goal
    goal_pose = get_pose_stampted(nav, {"x": 3.5, "y": 1.0}, 1.57)
    nav.goToPose(goal_pose)

    # wait for task to be completed
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # print(feedback)

    waypoints = [
        get_pose_stampted(nav, {"x": 3.5, "y": 2.0}, 3.14),
        get_pose_stampted(nav, {"x": 0.0, "y": 2.0}, -1.57),
        get_pose_stampted(nav, {"x": 0.0, "y": 0.0}, 0.0)
    ]

    nav.followWaypoints(waypoints)

    # wait for task to be completed
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # print(feedback)

    print(nav.getResult())

    rclpy.shutdown()

if __name__ == '__main__':
    main()

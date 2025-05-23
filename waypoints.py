# Based on example code from
# https://www.theconstruct.ai/ros2-qa-how-to-follow-waypoints-using-nav2-232/
# Accessed 02-05-2025

import time
from copy import deepcopy
from math import pi

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
#from nav2_msgs.msg import TaskResult


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    # pi/2 = towards grid
    # 0.0 = Left of grid
    # -pi/2 = Away from grid
    # pi = Right of grid

    inspection_route = [ # simulation points
        [-0.05, 1.0, pi/2],
        [-0.05, 2.0, pi/2],
        [-0.05, 3.0, pi/2],
        [-0.05, 4.5, pi/2],
        [-0.05, 6.0, pi/2],
        [-0.05, 7.5, pi/2],
        [-0.05, 9.0, pi/2],
        [-0.05, 7.5, pi/2],
        [-0.05, 6.0, pi/2],
        [-0.05, 4.5, pi/2],
        [-0.05, 3.0, pi/2],
        [-0.05, 1.5, pi/2],
        [-0.05, 0.0, pi/2],
        # Up and down first row
        [-0.05, 0.0, 0.0], # Rotate 90 degrees left
        [-3.0,  0.0, 0.0],
        [-6.0,  0.0, 0.0],
        # Down and up second row
        [-6.0, 0.0, pi/2], # Rotate forward
        [-6.0, 1.0, pi/2],
        [-6.0, 2.0, pi/2],
        [-5.9, 3.0, pi/2],
        [-5.9, 4.5, pi/2],
        [-5.9, 6.0, pi/2],
        [-5.9, 7.5, pi/2],
        [-5.9, 6.0, pi/2],
        [-5.9, 4.5, pi/2],
        [-5.9, 3.0, pi/2],
        [-6.0, 1.5, pi/2],
        [-6.0, 0.0, pi/2],
        ]



    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.707
    initial_pose.pose.orientation.w = 0.707
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    # navigator.waitUntilNav2Active()


    i = 0
    while rclpy.ok():
        inspection_route = inspection_route[i:]
        # Send our route
        inspection_points = []
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        for pt in inspection_route:
            inspection_pose.pose.position.x = pt[0]
            inspection_pose.pose.position.y = pt[1]
            if pt[2] == 0.0: # Actual -pi
                inspection_pose.pose.orientation.z = 1.0
                inspection_pose.pose.orientation.w = 0.0
            elif pt[2] == pi/2:
                inspection_pose.pose.orientation.z = 0.707
                inspection_pose.pose.orientation.w = 0.707
            elif pt[2] == -pi/2:
                inspection_pose.pose.orientation.z = -0.707
                inspection_pose.pose.orientation.w = 0.707
            elif pt[2] == pi: # Actual 0.0
                inspection_pose.pose.orientation.z = 0.0
                inspection_pose.pose.orientation.w = 1.0
            inspection_points.append(deepcopy(inspection_pose))
        # nav_start = navigator.get_clock().now()
        navigator.followWaypoints(inspection_points)

        # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
        # Simply print the current waypoint ID for the demonstation
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 10 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Inspection of shelves complete!')
            exit(0)
        elif result == TaskResult.CANCELED:
            print('Inspection of shelving was canceled.')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Inspection of shelving failed!')
            

        # go back to start
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # navigator.goToPose(initial_pose)
        # if navigator.isTaskComplete():
        #     navigator.lifecycleShutdown()
            
        #     exit(0)




if __name__ == '__main__':
    main()
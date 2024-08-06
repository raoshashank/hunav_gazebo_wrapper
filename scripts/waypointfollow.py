#!/usr/bin/env python3
#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import sys
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import rclpy.utilities
import yaml
def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    waypoint_file = rclpy.utilities.remove_ros_args(args)[1]
    print(waypoint_file)
    with open(waypoint_file,'r') as f:
        waypoints = yaml.safe_load(f)
    
    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 3.45
    # initial_pose.pose.position.y = 2.15
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    while rclpy.ok():

        # Send our route
        points = []
        waypoint_pose = PoseStamped()
        waypoint_pose.header.frame_id = 'map'
        waypoint_pose.header.stamp = navigator.get_clock().now().to_msg()
        waypoint_pose.pose.orientation.z = 1.0
        waypoint_pose.pose.orientation.w = 0.0
        for pt in waypoints:
            waypoint_pose.pose.position.x = pt['position']['x']
            waypoint_pose.pose.position.y = pt['position']['y']
            points.append(deepcopy(waypoint_pose))
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(points)

        # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
        # Simply print the current waypoint ID for the demonstation
        i = 0
        while not navigator.isNavComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(points)))

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('SUCCEEDED Waypoint following')
        elif result == TaskResult.CANCELED:
            print('Waypoint following CANCELLED')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Waypoint following FAILED')

        # go back to start
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # navigator.goToPose(initial_pose)
        while not navigator.isNavComplete:
            pass
if __name__ == '__main__':
    main()
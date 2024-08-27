#!/usr/bin/env python3

import sys
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml

class WaypointNavigator(BasicNavigator):
    def __init__(self, waypoints):
        super().__init__('waypoint_navigator')
        # Load waypoints from file
        self.robot_poses = waypoints
        self.get_logger().set_level(level=LoggingSeverity.DEBUG)
        # Subscribe to /amcl_pose to set the initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.robot_poses['initial_pose']['x']
        initial_pose.pose.position.y = self.robot_poses['initial_pose']['y']
        initial_quat = Rotation.from_euler('xyz',[0,0,self.robot_poses['initial_pose']['yaw']],degrees=False).as_quat()
        initial_pose.pose.orientation.x = initial_quat[0]
        initial_pose.pose.orientation.y = initial_quat[1]
        initial_pose.pose.orientation.z = initial_quat[2]
        initial_pose.pose.orientation.w = initial_quat[3]
        self.setInitialPose(initial_pose)
        self.waitUntilNav2Active()
        #self.navigate_waypoints()
    
    # def _amclPoseCallback(self, msg):
    #     self.initial_pose_received = True
    #     self.initial_pose = msg
    #     self.debug(f'Received amcl pose: {msg}')
    #     return
    
    # def _waitForInitialPose(self):
    #     while not self.initial_pose_received:
    #         self.info('Waiting for amcl_pose to be received')
    #         rclpy.spin_once(self, timeout_sec=1.0)
    #     return

    def navigate_waypoints(self):
        while rclpy.ok():
            points = []
            waypoint_pose = PoseStamped()
            waypoint_pose.header.frame_id = 'map'
            waypoint_pose.header.stamp = self.get_clock().now().to_msg()
            for pt in self.robot_poses['waypoints']:
                waypoint_pose.pose.position.x = pt['position']['x']
                waypoint_pose.pose.position.y = pt['position']['y']
                points.append(deepcopy(waypoint_pose))

            nav_start = self.get_clock().now()

            # Commented out for safety; you can uncomment and use it
            self.followWaypoints(points)

            # # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
            # # Simply print the current waypoint ID for the demonstration
            i = 0
            while not self.isTaskComplete():
                i = i + 1
                feedback = self.getFeedback()
                if feedback and i % 5 == 0:
                    self.get_logger().info('Executing current waypoint: ' +
                        str(feedback.current_waypoint + 1) + '/' + str(len(points)))

            result = self.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('SUCCEEDED Waypoint following')
                rclpy.shutdown()
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Waypoint following CANCELLED')
                rclpy.shutdown()
            elif result == TaskResult.FAILED:
                self.get_logger().error('Waypoint following FAILED')
                rclpy.shutdown()
                
            while not self.isTaskComplete():
                pass

def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    waypoint_file = rclpy.utilities.remove_ros_args(args)[1]
    with open(waypoint_file,'r') as f:
        waypoints = yaml.safe_load(f)

    waypoint_navigator = WaypointNavigator(waypoints)
    rclpy.spin(waypoint_navigator)

    waypoint_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

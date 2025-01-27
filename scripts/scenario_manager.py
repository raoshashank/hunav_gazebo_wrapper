#!/usr/bin/env python3
import rclpy,os
import rclpy.logging
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Int32
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from hunav_msgs.msg import Agent,Agents
from hunav_msgs.msg import PauseNavs,PauseNav
from rclpy.qos import QoSProfile
from collections import defaultdict
import json,yaml
import numpy as np
import signal
import sys
def is_near(pose1,pose2,thresh = 1.0):
    if isinstance(pose1,Pose):
        pose_1 = {'x':pose1.position.x,'y':pose1.position.y}
    else:
        pose_1 = pose1
    if isinstance(pose2,Pose):
        pose_2 = {'x':pose2.position.x,'y':pose2.position.y}
    else:
        pose_2 = pose2
        
    dist = np.sqrt((pose_1['x']-pose_2['x'])**2+(pose_1['y']-pose_2['y'])**2)
    #print(f"Distance:{dist}")
    if dist<thresh:
        return True
    return False

class ScenarioManager(Node):
    '''
        This node tracks the robot position, and pauses and unpauses human 
        navigation to make it more likely for a scenario to happen. 
        Default behavior:
            Pause the human navigation one-goal before the node where the human and 
            the robot are supposed to meet, until the robot arrives at the node preceding the interaction node
    '''
    def __init__(self,scenario_file,robot_poses_file):
        super().__init__('timing_manager')
        self.publisher_ = self.create_publisher(PauseNavs, 'pause_nav', 10)
        qos = QoSProfile(depth=10)
        self.robot_sub = self.create_subscription(Agent,'/robot_states',self.robotStateCallback,qos)
        #self.robot_status_sub = self.create_subscription(Int32,'/navigation_status',self.robotStatusCallback,qos)
        self.human_sub = self.create_subscription(Agents,'/human_states',self.humanStateCallback,qos)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.met_robot = {}
        self.robot_current_goal_index = 0
        self.robot_position = None

        with open(scenario_file,'r') as f:
            scenario = json.load(f)
            
        scenario_trajectories = scenario['trajectories']
        self.robot_goals = scenario_trajectories['robot'][1:]
        self.interaction_points = scenario['interaction_points']
        self.trajectories = scenario_trajectories
        with open(robot_poses_file,'r') as f:
            wrg = yaml.safe_load(f)
        self.world_robot_goals = wrg['waypoints']
        
        self.human_goals = defaultdict(lambda:[])
        for agent_name,traj in scenario_trajectories.items():
            goals = traj.copy()
            if agent_name!='robot':
                if len(traj)!=1:
                    goals = goals[1:]
                self.human_goals[agent_name] = goals
        self.pause_nav = {} # which humans have to be navigation-paused
        self.pause_msg = PauseNavs()

    def robotStateCallback(self,msg):
        '''
        Track which waypoint the robot is currently heading towards
        '''
        next_goal = self.world_robot_goals[self.robot_current_goal_index]['position']
        if is_near(next_goal,{'x':msg.position.position.x,'y':msg.position.position.y},1.5) and self.robot_current_goal_index!=len(self.robot_goals)-1:
            self.robot_current_goal_index+=1
        self.robot_position = msg.position
        #self.get_logger().info(f"current robot goal: {self.robot_goals[self.robot_current_goal_index]}")
        #print(f"Current goal index:{self.robot_current_goal_index}")
        
    def humanStateCallback(self,msg): 
        '''
        self.pause_nav = DefaultDict(False)
        Track which waypoint the human is currently heading to 
        For each human: 
            if the human NOT has met the robot already
                if the point corresponding to human_robot_first_intersection for this human is the next goal
                    If this point is not the next goal for the robot
                        self.pause_nav[human.id] = True
        '''
        #self.pause_nav = defaultdict(bool)
        self.pause_msg.agents = []
        if self.robot_position!=None:
            for human in msg.agents:
                if len(human.goals)==0 or (len(human.goals) == 1 and is_near(human.position,human.goals[0],human.goal_radius)): #last goal reached or no goal
                    # print(f"Human {human.id} has reached their final goal ({len(self.human_goals)}).")
                    continue
                self.pause_msg.agents.append(PauseNav(id = human.id, pause_nav = False))
                goal_index = len(self.human_goals[human.name])-len(human.goals)
                lstr0 = f"{human.id}. Goal index:{goal_index}, goals:{human.goals}, current goals:{self.human_goals[human.name]}"
                try:
                    lstr1 = f"Next goal for agent {human.id}: {self.human_goals[human.name][goal_index]}, IP: {self.interaction_points[human.name]}"
                except IndexError as e:
                    print(e)
                    # print(human.id)
                    # print(self.human_goals[human.name])
                    # print(human.goals)
                    # print(goal_index)
                    # print(self.interaction_points[human.name])
                #print(lstr0 + lstr1)
                
                #only manipulate the human's movement for RegularNav and when not in a group and when robot is not nearby
                if human.behavior_state == 0 and human.group_id == -1 and (not (is_near(human.position,self.robot_position))): 
                    #self.get_logger().log(f"Can manage {human.name}'s behavior",LoggingSeverity.INFO,throttle_duration_sec = 2.0)
                    if (self.interaction_points[human.name] == self.human_goals[human.name][goal_index]) or (self.interaction_points[human.name] == self.trajectories[human.name][0]): #is the human's next goal the interaction node, or the human is already at the interaction node at spawn.
                        #self.get_logger().log("human's next goal is the interaction node",LoggingSeverity.INFO,throttle_duration_sec = 2.0)
                        if self.robot_goals[self.robot_current_goal_index] != self.interaction_points[human.name]:# or (self.robot_goals[self.robot_current_goal_index] == self.interaction_points[human.name] and not(is_near(self.robot_position,self.robot_next_goal,1.0))): #is the robot's next goal the interaction node?
                            #self.get_logger().log("robot's next goal is NOT the interaction node",LoggingSeverity.INFO,throttle_duration_sec = 2.0)
                            if self.robot_current_goal_index<self.robot_goals.index(self.interaction_points[human.name]): #has the robot already passed the interaction point?
                                #self.get_logger().log("Robot hasn't passed the interaction point",LoggingSeverity.INFO,throttle_duration_sec = 2.0)
                                self.pause_msg.agents[-1].pause_nav = True #wait for the robot
                                lstr2 = f"Pausing agent {human.id}, interaction node is: {self.interaction_points[human.name]}, current robot goal: {self.robot_goals[self.robot_current_goal_index]}"
                                #self.get_logger().log(lstr2,LoggingSeverity.INFO,throttle_duration_sec = 2.0)
                                #print(lstr2)                    
    def timer_callback(self):
        self.publisher_.publish(self.pause_msg)
        #self.get_logger().info('Publishing')

def main(args=None):
    rclpy.init(args=args)
    with open(os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),'config','launch_params.yaml'),'rb') as f:
        config_file = yaml.safe_load(f)
        
    scenario_file = config_file['scenario']
    robot_poses_file = config_file['robot_poses']
    
    scenario_manager = ScenarioManager(scenario_file,robot_poses_file)

    rclpy.spin(scenario_manager)
    
    def signal_handler(sig, frame):
        scenario_manager.get_logger().info('Shutting down gracefully...')
        scenario_manager.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    scenario_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
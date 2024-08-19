#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from hunav_msgs.msg import Agent,Agents
from hunav_msgs.msg import PauseNavs,PauseNav
from rclpy.qos import QoSProfile
from collections import defaultdict
import json,yaml
import numpy as np
def is_near(pose1,pose2,thresh = 0.5):
    if isinstance(pose1,Pose):
        pose_1 = {'x':pose1.position.x,'y':pose1.position.y}
    else:
        pose_1 = pose1
    if isinstance(pose2,Pose):
        pose_2 = {'x':pose2.position.x,'y':pose2.position.y}
    else:
        pose_2 = pose2
        
    dist = np.sqrt((pose_1['x']-pose_2['x'])**2+(pose_1['y']-pose_2['y'])**2)
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
        self.human_sub = self.create_subscription(Agents,'/human_states',self.humanStateCallback,qos)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.met_robot = {}
        self.robot_current_goal_index = 0
        self.robot_position = None
         #assume this is initialized from some file
        with open(scenario_file,'r') as f:
            scenario = json.load(f)
            
        scenario_trajectories = scenario['trajectories']
        self.robot_goals = scenario_trajectories['robot'][1:]
        self.interaction_points = scenario['interaction_points']
        with open(robot_poses_file,'r') as f:
            wrg = yaml.safe_load(f)
        self.world_robot_goals = wrg['waypoints']
        # 'world_coords': [robot_poses['initial_pose']] + robot_poses['waypoints']
        #}
        self.human_goals = {}
        for agent_name,traj in scenario_trajectories.items():
            goals = traj.copy()
            if agent_name!='robot':
                if len(traj)!=1:
                    goals = goals[1:]
                self.human_goals[agent_name] = goals
            
            
        #self.human_robot_first_intersection = {} #key: ID, value: The first node in the scene graph where the human and robot meet.
        
        # for agent,points in scenario['trajectories'].items():
        #      if agent!='robot':
        #          self.human_goals[id] = points
                
        self.pause_nav = {} # which humans have to be navigation-paused
        self.pause_msg = PauseNavs()
          
    def robotStateCallback(self,msg): #COMPLETE THIS FUNCTION
        '''
        Track which waypoint the robot is currently heading towards
        '''
        next_goal = self.world_robot_goals[self.robot_current_goal_index]['position']
        if is_near(next_goal,{'x':msg.position.position.x,'y':msg.position.position.y}) and self.robot_current_goal_index!=len(self.robot_goals)-1:
            self.robot_current_goal_index+=1
        self.robot_position = msg.position
        #self.get_logger().info(f"current robot goal: {self.robot_goals[self.robot_current_goal_index]}")
        # print(f"Current goal index:{self.robot_current_goal_index}")
        
    def humanStateCallback(self,msg): #COMPLETE THIS FUNCTION
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
                self.pause_msg.agents.append(PauseNav(id = human.id, pause_nav = False))
                goal_index = len(self.human_goals[human.name])-len(human.goals)
                #lstr1 = f"Next goal for agent {human.id}: {self.human_goals[human.name][goal_index]}, IP: {self.interaction_points[human.name]}"
                #self.get_logger().info(lstr1)
                
                if human.behavior_state == 0 and human.group_id == -1 and (not (is_near(human.position,self.robot_position))): #only manipulate the human's movement for RegularNav and when not in a group and when robot is not nearby
                    
                    if self.interaction_points[human.name] == self.human_goals[human.name][goal_index]: #is the human's next goal the interaction node?
                        
                        if self.robot_goals[self.robot_current_goal_index] != self.interaction_points[human.name]: #is the robot's next goal the interaction node?
                            
                            if self.robot_current_goal_index<self.robot_goals.index(self.interaction_points[human.name]): #has the robot already passed the interaction point?
                                
                                self.pause_msg.agents[-1].pause_nav = True #wait for the robot
                                lstr2 = f"Pausing agent {human.id}, interaction node is: {self.interaction_points[human.name]}, current robot goal: {self.robot_goals[self.robot_current_goal_index]}"
                                self.get_logger().info(lstr2)
    def timer_callback(self):
        self.publisher_.publish(self.pause_msg)
        #self.get_logger().info('Publishing')

def main(args=None):
    rclpy.init(args=args)

    scenario_file = '/home/shashank/code/packages/SoNaS/scenarios/first_exp_response_traj.json'
    robot_poses_file = '/home/shashank/catkin_ws/src/hunav_gazebo_wrapper/config/robot_poses.yaml'
    
    scenario_manager = ScenarioManager(scenario_file,robot_poses_file)

    rclpy.spin(scenario_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scenario_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
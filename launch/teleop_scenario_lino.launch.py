from os import path
import os
from os import environ
from os import pathsep
from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, 
                            DeclareLaunchArgument, ExecuteProcess, Shutdown, 
                            RegisterEventHandler, TimerAction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, 
                            PythonExpression, EnvironmentVariable)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
import yaml
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd

def generate_launch_description():
    ld = LaunchDescription()
    
    #env vars
    model, plugin, media = GazeboRosPaths.get_paths()
    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']
    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'models',
    ])
    set_env_gazebo_model = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), my_gazebo_models]
    )
    set_env_gazebo_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH', 
        value=[EnvironmentVariable('GAZEBO_RESOURCE_PATH'), my_gazebo_models]
    )
    set_env_gazebo_plugin = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', 
        value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH'), plugin]
    )
    ld.add_action(set_env_gazebo_model)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(set_env_gazebo_plugin)
    
    
       
    # Gazebo
    config_file_name = 'params.yaml' 
    hunav_gazebo_wrapper_pkg_dir = get_package_share_directory('hunav_gazebo_wrapper') 
    config_file = path.join(hunav_gazebo_wrapper_pkg_dir, 'config', config_file_name) 
        
    ##################HuNavSim##############
    ld.add_action(DeclareLaunchArgument(
        'configuration_file', default_value='custom_agents.yaml',
        description='Specify configuration file name in the cofig directory'
    ))
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_agent_manager'),
        'config',
        LaunchConfiguration('configuration_file')
    ])
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
    ) 
    ld.add_action(hunav_loader_node)
    #generate world
    declare_arg_world = DeclareLaunchArgument(
        'base_world', default_value='no_roof_small_warehouse.world',
        description='Specify world file name'
    )
    declare_gz_obs = DeclareLaunchArgument(
        'use_gazebo_obs', default_value='true',
        description='Whether to fill the agents obstacles with closest Gazebo obstacle or not'
    )
    declare_update_rate = DeclareLaunchArgument(
        'update_rate', default_value='1000.0',
        description='Update rate of the plugin'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='linorobot2',
        description='Specify the name of the robot Gazebo model'
    )
    declare_frame_to_publish = DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='Name of the global frame in which the position of the agents are provided'
    )
    declare_use_navgoal = DeclareLaunchArgument(
        'use_navgoal_to_start', default_value='false',
        description='Whether to start the agents movements when a navigation goal is received or not'
    )
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='ground_plane',
        description='list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them'
    )
    
    ld.add_action(declare_arg_world)
    ld.add_action(declare_gz_obs)
    ld.add_action(declare_update_rate)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_frame_to_publish)
    ld.add_action(declare_use_navgoal)
    ld.add_action(declare_ignore_models)

    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        #output='screen',
        parameters=[{'base_world':PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
         LaunchConfiguration('base_world')])},
        {'use_gazebo_obs': LaunchConfiguration('use_gazebo_obs')},
        {'update_rate': LaunchConfiguration('update_rate')},
        {'robot_name': LaunchConfiguration('robot_name')},
        {'global_frame_to_publish': LaunchConfiguration('global_frame_to_publish')},
        {'use_navgoal_to_start': LaunchConfiguration('use_navgoal_to_start')},
        {'ignore_models': LaunchConfiguration('ignore_models')}]
        #arguments=['--ros-args', '--params-file', conf_file]
    )
           
    generate_world_process = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )
    ld.add_action(generate_world_process)
    
    
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    ld.add_action(hunav_manager_node)

    
    ##############Gazebo##############
    world_path = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        'generatedWorld.world'
    ])
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Set "true" to increase messages written to terminal.'
    )
    ld.add_action(declare_arg_verbose)

    gzserver_cmd = [
        # use_nvidia_gpu,
        'gzserver ',
        '--pause ',
         #LaunchConfiguration('base_world'),
         world_path, 
        _boolean_command('verbose'), '',
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        '-s ', 'libgazebo_ros_state.so',
        '--ros-args',
        '--params-file', config_file,
    ]

    gzclient_cmd = [
        # use_nvidia_gpu,
        'gzclient',
        _boolean_command('verbose'), ' ',
    ]

    gzserver_process = ExecuteProcess(
        cmd=gzserver_cmd,
        output='screen',
        shell=True,
        on_exit=Shutdown(),
    )

    gzclient_process = ExecuteProcess(
        cmd=gzclient_cmd,
        output='screen',
        shell=True,
        on_exit=Shutdown(),
    )
    gazebo_launch_process = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[gzserver_process, gzclient_process],
                )
            ]
        )
    )
    ld.add_action(gazebo_launch_process)
    #######################################
    #spawn robot
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_name = 'linorobot2'
    robot_params_file = os.path.join(hunav_gazebo_wrapper_pkg_dir,'config','robot_poses.yaml')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    
    with open(robot_params_file,'r') as f:
        robot_poses = yaml.safe_load(f)
    
    initial_pose = robot_poses['initial_pose']
    
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-entity', robot_name,
            '-topic','robot_description',       
            '-x', str(initial_pose['x']),
            '-y', str(initial_pose['y']),
            '-z','0.1',
            '-Y', str(initial_pose['yaw']),
            ],
        output='screen',
    )
    robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py'])),
            launch_arguments={
                'use_sim_time': str(True),
                'publish_joints': 'false',
            }.items()
        )
    robot_spawn_process = RegisterEventHandler(
        OnProcessStart(
            target_action=gzclient_process,
            on_start=[
                LogInfo(msg='Gazebo launched, bringing robot after 2 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[robot_state_publisher_cmd, spawn_robot_cmd],
                )
            ]
        )
    )
    ld.add_action(robot_spawn_process)
    
    #Launch Scenario Manager
    scenario_manager_node = Node(
        package='hunav_gazebo_wrapper',
        executable='scenario_manager.py',
        output='screen'
    )
    ld.add_action(scenario_manager_node)
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')
    ld.add_action(declare_rviz_config_file_cmd)
    
    ld.add_action(declare_rviz_config_file_cmd)
    return ld
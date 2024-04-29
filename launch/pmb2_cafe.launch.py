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
from launch_pal.include_utils import include_launch_py_description
def generate_launch_description():

    # to activate the use of nvidia gpu
    use_nvidia_gpu = [
        '__NV_PRIME_RENDER_OFFLOAD=1 ',
        '__GLX_VENDOR_LIBRARY_NAME=nvidia ',
    ]

    # World generation parameters
    world_file_name = LaunchConfiguration('base_world')
    gz_obs = LaunchConfiguration('use_gazebo_obs')
    rate = LaunchConfiguration('update_rate')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    navgoal_topic = LaunchConfiguration('navgoal_topic')
    ignore_models = LaunchConfiguration('ignore_models')

    # Robot parameters
    namespace = LaunchConfiguration('robot_namespace')
    scan_model = LaunchConfiguration('laser_model')
    use_rgbd = LaunchConfiguration('rgbd_sensors')
    gz_x = LaunchConfiguration('gzpose_x')
    gz_y = LaunchConfiguration('gzpose_y')
    gz_z = LaunchConfiguration('gzpose_z')
    gz_R = LaunchConfiguration('gzpose_R')
    gz_P = LaunchConfiguration('gzpose_P')
    gz_Y = LaunchConfiguration('gzpose_Y')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )



    # agent configuration file
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_agent_manager'),
        'config',
        LaunchConfiguration('configuration_file')
    ])

    # Read the yaml file and load the parameters
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    # world base file
    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        world_file_name
    ])

    # the node looks for the base_world file in the directory 'worlds'
    # of the package hunav_gazebo_plugin directly. So we do not need to 
    # indicate the path
    # hunav_gazebo_worldgen_node = Node(
    #     package='hunav_gazebo_wrapper',
    #     executable='hunav_gazebo_world_generator',
    #     output='screen',
    #     parameters=[{'base_world': world_file},
    #     {'use_gazebo_obs': gz_obs},
    #     {'update_rate': rate},
    #     {'robot_name': robot_name},
    #     {'global_frame_to_publish': global_frame},
    #     {'use_navgoal_to_start': use_navgoal},
    #     {'navgoal_topic': navgoal_topic},
    #     {'ignore_models': ignore_models}]
    #     #arguments=['--ros-args', '--params-file', conf_file]
    # )
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[{'base_world': world_file},
        {'use_gazebo_obs': gz_obs},
        {'update_rate': rate},
        {'robot_name': robot_name},
        {'global_frame_to_publish': global_frame},
        {'use_navgoal_to_start': use_navgoal},
        {'ignore_models': ignore_models}]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )

    # Then, launch the generated world in Gazebo 
    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'models',
    ])
    pmb2_gazebo_model = PathJoinSubstitution([
        FindPackageShare('pmb2_description'),
        'share'
    ])
    config_file_name = 'params.yaml' 
    pkg_dir = get_package_share_directory('hunav_gazebo_wrapper') 
    config_file = path.join(pkg_dir, 'launch', config_file_name) 

    
    model, plugin, media = GazeboRosPaths.get_paths()
    #print('model:', model)

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    env = {
        'GAZEBO_MODEL_PATH': model,
        'GAZEBO_PLUGIN_PATH': plugin,
        'GAZEBO_RESOURCE_PATH': media
    }
    #print('env:', env)
    

    set_env_gazebo_model = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), my_gazebo_models,":/home/shashank/catkin_ws/install/pmb2_description/share:"]
    )
    set_env_gazebo_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH', 
        value=[EnvironmentVariable('GAZEBO_RESOURCE_PATH'), my_gazebo_models]
    )
    set_env_gazebo_plugin = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', 
        value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH'), plugin]
    )
        
    world_path = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        'generatedWorld.world' #'empty_cafe.world' #'pmb2_cafe.world'
    ])

    gzserver_cmd = [
        # use_nvidia_gpu,
        'gzserver ',
        #'--pause ',
        # Pass through arguments to gzserver
         world_path, 
        _boolean_command('verbose'), '',
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        '-s ', 'libgazebo_ros_state.so',
        '-s', 'libgazebo_ros_control.so',
        '--ros-args',
        '--params-file', config_file,
    ]

    gzclient_cmd = [
        #use_nvidia_gpu,
        'gzclient',
        _boolean_command('verbose'), ' ',
    ]

    gzserver_process = ExecuteProcess(
        cmd=gzserver_cmd,
        output='screen',
        #additional_env=env,
        shell=True,
        on_exit=Shutdown(),
        #condition=IfCondition(LaunchConfiguration('server_required')),
    )

    gzclient_process = ExecuteProcess(
        cmd=gzclient_cmd,
        output='screen',
        #additional_env=env,
        shell=True,
        on_exit=Shutdown(),
        #condition=IfCondition(LaunchConfiguration('server_required')),
    )

    

    # Finally, spawn the pmb2 robot in Gazebo

    # pmb2_gazebo_launch = PathJoinSubstitution([
    #     FindPackageShare("hunav_gazebo_wrapper"),
    #     "launch",
    #     "pmb2_pal.launch.py"
    # ],)

    # pmb2_gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([pmb2_gazebo_launch])
    # )
    pmb2_gazebo = include_launch_py_description(
        'hunav_gazebo_wrapper', ['launch', 'pmb2_pal.launch.py'])

    # Do not launch Gazebo until the world has been generated
    ordered_launch_event2 = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[gzserver_process, gzclient_process],
                )
            ]
        )
    )

    # hunav_manager node
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    metrics_file = PathJoinSubstitution([
        FindPackageShare('hunav_evaluator'),
        'config',
        LaunchConfiguration('metrics_file')
    ])

    # hunav_evaluator node
    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[metrics_file]
    )

    # DO NOT Launch this if any robot localization is launched
    static_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        output='screen',
        # arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
        arguments = "0 0 0 0 0 0 pmb2 base_footprint".split(' ')
    )

    declare_agents_conf_file = DeclareLaunchArgument(
        'configuration_file', default_value='agents.yaml',
        description='Specify configuration file name in the cofig directory'
    )
    declare_metrics_conf_file = DeclareLaunchArgument(
        'metrics_file', default_value='metrics.yaml',
        description='Specify the name of the metrics configuration file in the cofig directory'
    )
    declare_arg_world = DeclareLaunchArgument(
        'base_world', default_value='example_cafe.world',
        description='Specify world file name'
    )
    declare_gz_obs = DeclareLaunchArgument(
        'use_gazebo_obs', default_value='true',
        description='Whether to fill the agents obstacles with closest Gazebo obstacle or not'
    )
    declare_update_rate = DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='Update rate of the plugin'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='pmb2',
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
    declare_navgoal_topic = DeclareLaunchArgument(
        'navgoal_topic', default_value='goal_pose',
        description='Name of the topic in which navigation goal for the robot will be published'
    )
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='ground_plane cafe',
        description='list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them'
    )
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Set "true" to increase messages written to terminal.'
    )
    declare_arg_namespace = DeclareLaunchArgument('robot_namespace', default_value='',
            description='The type of robot')
    #DeclareLaunchArgument('gzpose', default_value='-x 0.0 -y 0.0 -z 0.1 -R 0.0 -P 0.0 -Y 1.57',
    #                      description='The robot initial position in the world')
    declare_arg_px = DeclareLaunchArgument('gzpose_x', default_value='0.0',
            description='The robot initial position in the X axis of the world')
    declare_arg_py = DeclareLaunchArgument('gzpose_y', default_value='0.0',
            description='The robot initial position in the Y axis of the world')
    declare_arg_pz = DeclareLaunchArgument('gzpose_z', default_value='0.25',
            description='The robot initial position in the Z axis of the world')
    declare_arg_pR = DeclareLaunchArgument('gzpose_R', default_value='0.0',
            description='The robot initial roll angle in the world')
    declare_arg_pP = DeclareLaunchArgument('gzpose_P', default_value='0.0',
            description='The robot initial pitch angle in the world')
    declare_arg_pY = DeclareLaunchArgument('gzpose_Y', default_value='0.0',
            description='The robot initial yaw angle in the world')
    declare_arg_laser = DeclareLaunchArgument('laser_model', default_value='sick-571', # Removed sick-571-gpu cause new pmb2 does not has it
            description='the laser model to be used')
    declare_arg_rgbd = DeclareLaunchArgument('rgbd_sensors', default_value='false',
            description='whether to use rgbd cameras or not')

    

    ld = LaunchDescription()

    # set environment variables
    ld.add_action(set_env_gazebo_model)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(set_env_gazebo_plugin)
    # Declare the launch arguments
    
    ld.add_action(declare_agents_conf_file)
    ld.add_action(declare_metrics_conf_file)
    ld.add_action(declare_arg_world)
    ld.add_action(declare_gz_obs)
    ld.add_action(declare_update_rate)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_frame_to_publish)
    ld.add_action(declare_use_navgoal)
    ld.add_action(declare_ignore_models)
    ld.add_action(declare_arg_verbose)
    
    #ld.add_action(declare_navgoal_topic)
    # ld.add_action(declare_arg_namespace)
    # ld.add_action(declare_arg_laser)
    # ld.add_action(declare_arg_rgbd)
    # ld.add_action(declare_arg_px)
    # ld.add_action(declare_arg_py)
    # ld.add_action(declare_arg_pz)
    # ld.add_action(declare_arg_pR)
    # ld.add_action(declare_arg_pP)
    # ld.add_action(declare_arg_pY)

    # Generate the world with the agents
    # launch hunav_loader and the WorldGenerator
    # 2 seconds later
    ld.add_action(hunav_loader_node)
    # hunav behavior manager node
    #ld.add_action(hunav_manager_node)
    ld.add_action(ordered_launch_event)

    # hunav behavior manager node
    ld.add_action(hunav_manager_node)
    # hunav evaluator
    #ld.add_action(hunav_evaluator_node)

    # launch Gazebo after worldGenerator 
    # (wait a bit for the world generation) 
    # ld.add_action(gzserver_process)
    ld.add_action(ordered_launch_event2)
    # ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(spawn_turtlebot_cmd)
    # ld.add_action(gzclient_process)

    # spawn robot in Gazebo
    ld.add_action(pmb2_gazebo)

    # print(environ['GAZEBO_MODEL_PATH'])
    # print(environ['GAZEBO_RESOURCE_PATH'])
    # print(environ['GAZEBO_PLUGIN_PATH'])

    return ld

    


# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd

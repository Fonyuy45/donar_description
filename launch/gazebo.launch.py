#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import TimerAction

def generate_launch_description():
    
    pkg_share = get_package_share_directory('donar_description')
    
    # SDF file path
    #sdf_file = os.path.join(pkg_share, 'models', 'donar_robot', 'model.sdf')


    # 2. URDF/Xacro Path (For RViz Visualization)
    xacro_file = os.path.join(pkg_share, 'urdf', 'donar_robot.xacro')
    
    # Process Xacro into URDF
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}
    
    # World file
    world_file = os.path.join(pkg_share, 'worlds', 'empty_world.sdf')
    
    # Set Gazebo resource path
    set_gazebo_path = SetEnvironmentVariable(
	    name='GZ_SIM_RESOURCE_PATH',
	    value=os.path.dirname(pkg_share)
	)
    
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': world_file + ' -r'}.items()
    )
    
    # Spawn Robot from SDF
    spawn_robot = Node(
	    package='ros_gz_sim',
	    executable='create',
	    arguments=[
		'-topic', 'robot_description',
		'-name', 'donar_robot',
		'-x', '0', '-y', '0', '-z', '0.15'
	    ],
	    output='screen'
       )
    # 5. Robot State Publisher (Reads URDF, publishes TFs for RViz)
    robot_state_publisher = TimerAction(
	    period=3.0,
	    actions=[Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		output='screen',
		parameters=[robot_description, {'use_sim_time': True}]
	    )]
	)
    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # Differential Drive
            '/donar_robot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/donar_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            
            # Joint States
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',

            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 7. RViz2
    rviz_config = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz') # Optional: if you have a saved config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
            set_gazebo_path,
            gazebo,
            spawn_robot,
            robot_state_publisher, 
            bridge,
            rviz_node              
        ])



#!/usr/bin/env python3
"""

Launch order:
  1. Set environment variables
  2. Start Gazebo
  3. Start Robot State Publisher (must be up before spawn so /robot_description exists)
  4. Spawn robot into Gazebo (with small delay to let Gazebo settle)
  5. Start ROS-Gazebo bridge
  6. Start RViz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Package paths

    pkg_share = get_package_share_directory('donar_description')

    # Xacro file → processed into URDF and published on /robot_description
    xacro_file = os.path.join(pkg_share, 'urdf', 'donar_robot.xacro')

    # World SDF file loaded by Gazebo
    world_file = os.path.join(pkg_share, 'worlds', 'empty_world.sdf')

    # RViz config (optional — node still launches without it)
    rviz_config = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    # Robot description (Xacro → URDF string)

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # 1. Environment variables
    # Tell Gazebo where to find the robot meshes and models.
    # os.path.dirname(pkg_share) points one level above the package share,
    # so Gazebo can discover the donar_description package models folder.
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_share)
    )

    # 2. Gazebo Ignition Fortress
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        # '-r' auto-starts the simulation (without it Gazebo opens paused)
        launch_arguments={'gz_args': world_file + ' -r'}.items()
    )

    # 3. Robot State Publisher
    # Reads the URDF and publishes:
    # Must start before spawn_robot so /robot_description is available.
    # use_sim_time: True → syncs to /clock published by Gazebo bridge.

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Spawn robot into Gazebo
    # Reads URDF from /robot_description topic and creates the robot entity.
    # Small delay via TimerAction to allow Gazebo to fully initialize first.

    spawn_robot = TimerAction(
        period=2.0,  # seconds — wait for Gazebo to be ready
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',  # source URDF from topic
                    '-name', 'donar_robot',          # entity name in Gazebo
                    '-x', '0',                       # spawn position X
                    '-y', '0',                       # spawn position Y
                    '-z', '0.15'                     # spawn height (above ground)
                ],
                output='screen'
            )
        ]
    )

    # 5. ROS <-> Gazebo Bridge (ros_gz_bridge)
    # use_sim_time: True → bridge itself uses /clock for timestamps
    #

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[

            # -- Simulation clock (Gazebo → ROS 2) ---------------------------
            # Provides /clock so all nodes can sync to sim time
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

            # -- Velocity commands (ROS 2 → Gazebo) --------------------------
            # Differential drive plugin reads this to move the robot.
            # Use ] not @ to avoid the bridge echoing commands back into ROS 2.
            '/donar_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',

            # -- Odometry (Gazebo → ROS 2) ------------------------------------
            # Differential drive plugin publishes wheel odometry
            '/donar_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',

            # -- Joint states (Gazebo → ROS 2) --------------------------------
            # Wheel joint positions/velocities for robot_state_publisher
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',

            # -- Transforms (Gazebo → ROS 2) ----------------------------------
            # Dynamic TFs (e.g. wheel joints) from the physics simulation
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # /tf_static uses TRANSIENT_LOCAL QoS override (see parameters below)
            '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

            # -- LiDAR (Gazebo → ROS 2) ---------------------------------------
            '/donar_robot/laser/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',

            # -- Depth camera (Gazebo → ROS 2) --------------------------------
            '/donar_robot/depth_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/donar_robot/depth_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/donar_robot/depth_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/donar_robot/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',

            # -- IMU (Gazebo → ROS 2) -----------------------------------------
            '/donar_robot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Override /tf_static publisher durability to TRANSIENT_LOCAL so
            # RViz and tf2 listeners receive static transforms even if they
            # subscribe after the bridge has already published them.
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'qos_overrides./tf_static.publisher.reliability': 'reliable',
        }]
    )

    # 7. RViz2
    # Visualization — loads saved config if it exists, otherwise opens blank.
    # use_sim_time: True → RViz syncs playback to Gazebo simulation clock.
  
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

  
    # Launch Description — order matters for dependencies

    return LaunchDescription([
        set_gazebo_resource_path,   # env vars first
        gazebo,                     # start simulator
        robot_state_publisher,      # publish /robot_description before spawn
        spawn_robot,                # spawn into Gazebo (delayed 2s)
        bridge,                     # ROS <-> Gazebo data bridge
        rviz_node,                  # visualization
    ])
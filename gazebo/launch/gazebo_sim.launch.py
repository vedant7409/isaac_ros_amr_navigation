#!/usr/bin/env python3

"""
Gazebo Simulation Launch File
Spawns robot in Gazebo with all sensors
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('isaac_ros_amr_navigation')
    gz_sim_share = FindPackageShare('ros_gz_sim')
    
    # Paths
    world_file = PathJoinSubstitution([
        pkg_share, 'gazebo', 'worlds', 'empty.world'
    ])
    
    urdf_file = PathJoinSubstitution([
        pkg_share, 'robot_description', 'urdf', 'bot.urdf.xacro'
    ])
    
    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Gazebo world file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )
    
    # Robot description
    robot_description = Command(['xacro ', urdf_file])
    
    # Gazebo server
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': ['-r -s ', LaunchConfiguration('world')],
        }.items()
    )
    
    # Gazebo client (GUI)
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': '-g'
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Bridge for camera topics
    bridge_camera_left = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )
    
    bridge_camera_right = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/right/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )
    
    # Bridge for other sensors
    bridge_sensors = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gui_arg,
        gz_server,
        gz_client,
        robot_state_publisher,
        spawn_robot,
        bridge_camera_left,
        bridge_camera_right,
        bridge_sensors,
    ])


if __name__ == '__main__':
    generate_launch_description()

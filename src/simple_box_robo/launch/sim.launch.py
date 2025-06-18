import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_box_robo')
    
    # Generate URDF from xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('simple_box_robo'),
            'model',
            'robot.urdf.xacro'
        ])
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    return LaunchDescription([
        # Launch Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r', 
                 PathJoinSubstitution([
                     FindPackageShare('simple_box_robo'),
                     'worlds',
                     'obstacle_world.world'
                 ])],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        # Spawn the robot
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'simple_box_robo',
                '-x', '-1.0',
                '-y', '0.0',
                '-z', '0.5',
                '-Y', '0.0',
            ],
            output='screen'
        ),

        # ROS-Ignition Bridge (only for odometry and TF)
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
            ],
            output='screen'
        )

    ])
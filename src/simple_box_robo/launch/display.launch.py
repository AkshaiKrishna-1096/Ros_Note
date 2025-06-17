import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your URDF file
    urdf_file = os.path.join(
        get_package_share_directory('simple_box_robo'),  # Replace with your package name
        'model',
        'robot.urdf.xacro'
    )
    
    # RViz config file
    rviz_config = os.path.join(
        get_package_share_directory('simple_box_robo'),  # Replace with your package name
        'rviz',
        'config.rviz'
    )

    # Generate URDF from xacro
    robot_description = Command([
        'xacro ', urdf_file
    ])

    return LaunchDescription([
        # Robot State Publisher
        # Add this parameter to your robot_state_publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 30.0  # Add this line
            }],
        ),

        # Joint State Publisher GUI (for manual joint control)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Teleop velocity command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=['/safe_cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        ),

        # Odometry
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odom_bridge',
            output='screen',
            arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        ),

        # Joint states
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='joint_state_bridge',
            output='screen',
            arguments=['/joint_states@sensor_msgs/msg/JointState]gz.msgs.Model'],
        ),

        # Lidar
        # Lidar - add transform remapping
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            output='screen',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            ],
        ),

        # Camera
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            output='screen',
            arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        ),
    ])

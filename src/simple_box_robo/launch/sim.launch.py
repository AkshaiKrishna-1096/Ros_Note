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

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
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

        # Add LiDAR bridge
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/keyboard_cmd@geometry_msgs/msg/Twist@ignition.msgs.Twist',  # Add this line
                '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                '/world/empty/model/simple_box_robo/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model'
            ],
            output='screen'
        ),

        # Add Obstacle Avoidance Node
        Node(
            package='simple_box_robo',
            executable='obstacle_avoidance_node.py',
            output='screen',
            remappings=[
                ('/scan', '/lidar')
            ]
        )

    ])
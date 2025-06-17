import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to your world and URDF files
    pkg_share = get_package_share_directory('simple_box_robo')  
    
    world_file = os.path.join(pkg_share, 'worlds', 'obstacle_world.world')
    urdf_file = os.path.join(pkg_share, 'model', 'robot.urdf.xacro')

    # Generate URDF from xacro
    robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_file])

    controller_config = os.path.join(
        get_package_share_directory('simple_box_robo'),
        'config',
        'controllers.yaml'
    )

    return LaunchDescription([
        # Launch Gazebo Fortress with the specified world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r', world_file],  # Changed 'gz' to 'ignition' for Fortress
            output='screen',
            shell=True
        ),

        # Publish robot description to ROS parameter server
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Spawn the robot into Gazebo
        Node(
            package='ros_gz_sim', 
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

        # Bridge ROS topics to Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/simple_box_robo/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
            ],
            output='screen',
            remappings=[
                ('/cmd_vel', '/model/simple_box_robo/cmd_vel')
            ],
            parameters=[{
                'qos_overrides./cmd_vel.publisher.reliability': 'reliable',
                'qos_overrides./model/simple_box_robo/cmd_vel.subscription.reliability': 'reliable'
            }]
        ),

        # Teleop Keyboard Node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',
            emulate_tty=True,
            remappings=[
                ('/cmd_vel', '/model/simple_box_robo/cmd_vel')  # Direct remapping
            ]
        ),

        # Add these nodes to your LaunchDescription
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, controller_config],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
            output='screen',
        ),

        Node(
            package='simple_box_robo',
            executable='obstacle_avoidance_node.py',
            name='obstacle_avoidance_node',
            output='screen',
        )

    ])
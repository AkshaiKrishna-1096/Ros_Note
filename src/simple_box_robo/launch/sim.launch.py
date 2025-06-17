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
            package='ros_gz_bridge',  # Changed from 'ros_gz_bridge' to 'ros_ign_bridge' for Fortress
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',  # Changed 'gz.msgs' to 'ignition.msgs'
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                # You might also want to add TF bridge if needed
                '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
            ],
            output='screen'
        ),
    ])
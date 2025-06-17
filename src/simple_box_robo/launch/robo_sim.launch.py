import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to your world and URDF files
    pkg_share = get_package_share_directory('your_pkg_name')  # Replace with your package name
    
    world_file = os.path.join(pkg_share, 'worlds', 'obstacle_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robo_model.urdf.xacro')

    # Generate URDF from xacro
    robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_file])

    return LaunchDescription([
        # Launch Gazebo with the specified world
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', '-r', world_file],  # '-r' runs simulation immediately
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
                '-name', 'my_robot',
                '-x', '0.0',  # Spawn position X
                '-y', '0.0',  # Spawn position Y
                '-z', '0.1',  # Spawn position Z (slightly above ground)
                '-Y', '0.0',  # Yaw orientation
            ],
            output='screen'
        ),

        # Optional: Bridge ROS topics to Gazebo (if using ROS control)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            ],
            output='screen'
        ),
    ])
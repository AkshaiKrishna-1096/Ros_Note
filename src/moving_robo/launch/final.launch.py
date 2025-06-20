import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # == Parameter ===
    roboName = 'simple_box_robo'
    packageName = 'moving_robo'

    modelPath = os.path.join(
        get_package_share_directory(packageName),
        'model',
        'robot.urdf.xacro'
    )

    worldPath = os.path.join(
        get_package_share_directory(packageName),
        'world',
        'world.world'
    )

    rvizConfig = os.path.join(
        get_package_share_directory(packageName),
        'config',
        'config.rviz'
    )

    robotDescription = ParameterValue(
        Command(['xacro ', modelPath]),
        value_type=str
    )
    

    # === Node ===
    rps = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robotDescription}
        ]
    )


    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfig]
    )

    spawnRobo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', roboName,
            '-x', '-1.0',
            '-y', '0.0',
            '-z', '0.5',
            '-Y', '0.0',
        ],
        output='screen'
    )


    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    safeStopper = Node(
        package=packageName,
        executable='safe_stopper',
        output='screen'
    )

    # == Execute Process ===
    gazeboLaunch = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-v', '4', '-r', worldPath
        ],
        output='screen'
    )

    # == BRIDGE ===
    bridgeLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(packageName),
                'launch',
                'bridge.launch.py'
            )
        )
    )

    # Add this with your other nodes
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_body'],
        output='screen'
    )

    return LaunchDescription([
        rps,
        jsp,
        rviz2,
        gazeboLaunch,
        spawnRobo,
        teleop,
        bridgeLaunch,
        safeStopper,
        static_transform
    ])

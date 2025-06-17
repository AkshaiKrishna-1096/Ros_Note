import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('simple_box_robo')
    
    return LaunchDescription([
        # Launch RViz visualization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_path, 'launch', 'display.launch.py')
            ])
        ),
        
        # Launch Gazebo simulation with robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_path, 'launch', 'sim.launch.py')
            ])
        ),
        
    ])

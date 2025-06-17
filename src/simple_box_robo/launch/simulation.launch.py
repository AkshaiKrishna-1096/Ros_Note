import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('your_pkg_name')
    
    return LaunchDescription([
        # Launch RViz visualization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_path, 'launch', 'robo_model.launch.py')
            ])
        ),
        
        # Launch Gazebo simulation with robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_path, 'launch', 'gazebo_sim.launch.py')
            ])
        ),
        
        # Launch teleop keyboard
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_path, 'launch', 'teleop.launch.py')
            ])
        ),
    ])
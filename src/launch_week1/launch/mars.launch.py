from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    numbers = Node(
        package="week1",
        executable="number_publisher"
    )

    squared = Node(
        package="week1",
        executable="squared_publisher"
    )

    ld.add_action(numbers)
    ld.add_action(squared)

    return ld

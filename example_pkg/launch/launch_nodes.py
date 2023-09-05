from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_pkg', # required
            executable='publisher', # required, name of python script that spins the node
            name='fun_publisher', # optional, useful for renaming nodes if you want to run multiple of the same node           
        ),
        Node(
            package='example_pkg',
            executable='subscriber',
            name='fun_subscriber'
        )
    ])
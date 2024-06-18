from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rov',
            namespace='rov',
            executable='rov',
            name='rov'
        ),
        Node(
            package='rov',
            namespace='bldc',
            executable='bldc',
            name='bldc'
        ),
        # Node(
        #     package='rov',
        #     executable='joy_2_cmd',
        #     name='mimic',
        # )
    ])
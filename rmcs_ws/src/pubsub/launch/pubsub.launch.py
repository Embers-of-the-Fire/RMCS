from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub',
            executable='publisher_node',
            name='publisher_node',
            output='screen'
        ),
        Node(
            package='pubsub',
            executable='subscriber_node',
            name='subscriber_node',
            output='screen'
        ),
    ])

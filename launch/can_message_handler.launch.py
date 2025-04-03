from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_message_handler',
            executable='can_message_publisher_node',
            name='can_message_publisher',
            parameters=[
                {'can_ids': [0x001, 0x002, 0x003]}
            ]
        ),
        Node(
            package='can_message_handler',
            executable='can_message_receiver_node',
            name='can_message_receiver',
            parameters=[
                {'target_can_id': 0x211},
                {'target_byte': 1},
                {'target_bit': 0}
            ]
        )
    ]) 
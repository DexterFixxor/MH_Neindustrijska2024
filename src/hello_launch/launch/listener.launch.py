from launch import LaunchDescription # ROS object that embeds a launch configuration
from launch_ros.actions import Node # ROS object that represents a Node to start


def generate_launch_description():
    """Builds a launch description."""
    ld = LaunchDescription()
    # Without the following the Launch System would just spawn a process
    # that would terminate immediately since no node has been specified
    node1 = Node(
        package='hello_launch',
        executable='hello_world',
        namespace="node1"
    )
    ld.add_action(node1)
    
    node2 = Node(
        package='hello_launch',
        executable='hello_world',
        namespace="node2"
    )
    ld.add_action(node2)
    
    # And this starts one node, just repeat the previous block of code
    # to start more nodes at once!
    return ld

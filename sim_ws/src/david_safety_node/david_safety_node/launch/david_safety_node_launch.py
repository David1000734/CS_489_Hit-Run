from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="david_safety_node",
        executable="david_safety_node",
    )

    ld.add_action(talker_node)

    return ld

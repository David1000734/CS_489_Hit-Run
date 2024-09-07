from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    v = LaunchConfiguration('v', default = '0.0')
    d = LaunchConfiguration('d', default = '0.0')

    ld = LaunchDescription()

    talker_node = Node(
        package="lab1_pkg",
        executable="talker",
        parameters=[{'v': v},
                    {'d': d}]
    )

    listener_node = Node(
        package="lab1_pkg",
        executable="relay",
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld
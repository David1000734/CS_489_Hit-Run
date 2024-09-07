from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    v = LaunchConfiguration('v')
    d = LaunchConfiguration('d')

    # v = DeclareLaunchArgument('v', default_value=float(17))
    # d = DeclareLaunchArgument('d', default_value=float(88))

    # TEMP
    if (v == None):
        v = float(5.3)
        d = float(32.4)

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
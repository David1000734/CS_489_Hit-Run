from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()
    
    pkg_node = Node(
        package = "pure_pursuit",
        executable = "pure_pursuit_node",
        parameters = [
            {'mode' : LaunchConfiguration('mode', default = 'sim')},
            {'speed' : LaunchConfiguration('speed', default = '1.0')}
        ]
    )

    pkg_marker = Node(
        package = "pure_pursuit",
        executable = "marker",
        parameters = [
            {'mode' : LaunchConfiguration('mode', default = 'sim')}
        ]
    )

    # Working on marker, comment for now
    # ld.add_action(pkg_node)
    ld.add_action(pkg_marker)

    return ld

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
            {'speed' : LaunchConfiguration('speed', default = '1.0')}
        ]
    )

    ld.add_action(pkg_node)

    return ld

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()
    
    pkg_node = Node(
        package = "gap_follow",
        executable = "reactive_node",
        parameters = [
            {'mode'  : LaunchConfiguration('mode', default = 'sim')},
            {'bubble': LaunchConfiguration('bubble', default = '2')},
            {'speed' : LaunchConfiguration('speed', default = '2.0')},
            {'gap'   : LaunchConfiguration('gap', default = '4')},
            {'dist'  : LaunchConfiguration('dist', default = '5.0')},
            {'change': LaunchConfiguration('change', default = '0.5')}
        ]
    )

    ld.add_action(pkg_node)

    return ld

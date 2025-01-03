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
            {'bubble': LaunchConfiguration('bubble', default = '0.1')},
            {'distp' : LaunchConfiguration('disp', default = '5')},
            {'speed' : LaunchConfiguration('speed', default = '2.0')},
            {'gap'   : LaunchConfiguration('gap', default = '4')},
            {'dist'  : LaunchConfiguration('dist', default = '5.0')},
            {'change': LaunchConfiguration('change', default = '0.5')},
            {'lowerX': LaunchConfiguration('lowerX', default = '180')},
            {'upperX': LaunchConfiguration('upperX', default = '900')}
        ]
    )

    ld.add_action(pkg_node)

    return ld

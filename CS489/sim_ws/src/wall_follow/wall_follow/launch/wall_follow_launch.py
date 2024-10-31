from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()

    speed = LaunchConfiguration('speed', default = '2.0')
    var_P = LaunchConfiguration('P', default = '2.0')
    var_I = LaunchConfiguration('I', default = '2.0')
    var_D = LaunchConfiguration('D', default = '2.0')
    mode = LaunchConfiguration('mode', default = 'sim')
    
    pkg_node = Node(
        package = "wall_follow",
        executable = "wall_follow_node",
        parameters = [
            {'speed' : speed},
            {'P' : var_P},
            {'I' : var_I},
            {'D' : var_D},
            {'mode' : mode}
        ]
    )

    ld.add_action(pkg_node)

    return ld

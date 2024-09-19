from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()

    ttc = LaunchConfiguration('ttc', default = '1.0')
    mode = LaunchConfiguration('mode', default = 'sim')
    student = LaunchConfiguration('student', default = 'david')

    package_name = [student, TextSubstitution(text = '_safety_node')]
    
    pkg_node = Node(
        package = package_name,
        executable = package_name,
        parameters = [
            {'ttc' : ttc},
            {'mode' : mode}
        ]
    )

    ld.add_action(pkg_node)

    return ld

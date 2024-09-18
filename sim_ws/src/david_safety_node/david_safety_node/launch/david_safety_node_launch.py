from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()

    ttc = LaunchConfiguration('ttc', default = '1.0')
    mode = LaunchConfiguration('mode', default = 'sim')
    student = LaunchConfiguration('student', default = 'david')
    # print(student)
    # student = "david"

    nodes = []

    # print("Start")
    # print(student)
    # if (IfCondition(LaunchConfiguration('student') == student)):
    if (student == "david"):
        nodes.append(Node(
            package="david_safety_node",
            executable="david_safety_node",
            parameters = [
                {'ttc' : ttc},
                {'mode' : mode}
            ]
        ))
    else:
        pass

    # match student:
    #     case "david":
    #         talker_node = Node(
    #             package="david_safety_node",
    #             executable="david_safety_node",
    #             parameters = [
    #                 {'ttc' : ttc},
    #                 {'mode' : mode}
    #             ]
    #         )
    #     case _:
    #         print("Nothing Ran...")

    for node in nodes:
        ld.add_action(node)

    return ld

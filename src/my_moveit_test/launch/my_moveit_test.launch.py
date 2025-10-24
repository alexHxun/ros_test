from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    my_node = Node(
        package="my_moveit_test",
        executable="my_moveit_test",
        output="screen",
        parameters=[
            moveit_config.to_dict(),  # robot_description 等
            {"planning_group": "panda_arm"}
        ]
    )

    return LaunchDescription([my_node])

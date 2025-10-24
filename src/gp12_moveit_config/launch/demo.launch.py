from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # 声明是否启用 RViz 参数
    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start RViz2"
        )
    ]

    use_rviz = LaunchConfiguration("use_rviz")

    moveit_config = (
        MoveItConfigsBuilder("motoman_gp12", package_name="gp12_moveit_config")
        .robot_description(file_path="config/motoman_gp12.urdf.xacro")
        .robot_description_semantic(file_path="config/motoman_gp12.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    ld = LaunchDescription(declared_arguments)

    # 各节点定义
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, "/home/alex/ros2_ws/src/gp12_moveit_config/config/ros2_controllers.yaml"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config = "/home/alex/ros2_ws/src/gp12_moveit_config/config/moveit.rviz"
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_rviz),
    )

    # 添加节点
    ld.add_action(static_tf)
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(arm_controller_spawner)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)

    return ld

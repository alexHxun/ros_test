from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("motoman_gp12", package_name="gp12_moveit_config")
        .robot_description(file_path="config/motoman_gp12.urdf.xacro")
        .robot_description_semantic(file_path="config/motoman_gp12.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])  # 明确指定规划管道
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    # 使用 MoveIt 的标准 demo launch（包含 robot_state_publisher、rviz 等）
    # 但我们需要自定义控制器部分，所以手动添加节点
    
    # 1. 单一 static_transform_publisher（world -> base_link）
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    # 2. ros2_control 节点
    ros2_controllers_path = "/home/alex/ros2_ws/src/gp12_moveit_config/config/ros2_controllers.yaml"
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    # 3. 控制器 spawner
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

    # 4. 确保控制器按顺序启动
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_controller_spawner],
        )
    )

    # 5. robot_state_publisher（只启动一个）
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # 6. move_group 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # 7. RViz
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
    )

    # 添加所有节点
    ld.add_action(static_tf)
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)

    return ld
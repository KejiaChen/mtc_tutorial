import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sensone_left = DeclareLaunchArgument("use_sensone_left", default_value="false")
    use_sensone_right = DeclareLaunchArgument("use_sensone_right", default_value="true")
    alter_finger_left = DeclareLaunchArgument("alter_finger_left", default_value="false")
    alter_finger_right = DeclareLaunchArgument("alter_finger_right", default_value="false")
    send_to_robot = DeclareLaunchArgument("send_to_robot", default_value="true")
    follower_ip = DeclareLaunchArgument("follower_ip", default_value="10.157.174.87")
    follower_port = DeclareLaunchArgument("follower_port", default_value="12345")
    publish_legacy_topic = DeclareLaunchArgument("publish_legacy_topic", default_value="false")
    start_move_group = DeclareLaunchArgument("start_move_group", default_value="false")
    sync_via_move_group = DeclareLaunchArgument("sync_via_move_group", default_value="true")

    moveit_config = (
        MoveItConfigsBuilder("dual_arm_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "use_sensone_left": LaunchConfiguration("use_sensone_left"),
                "use_sensone_right": LaunchConfiguration("use_sensone_right"),
                "alter_finger_left": LaunchConfiguration("alter_finger_left"),
                "alter_finger_right": LaunchConfiguration("alter_finger_right"),
            },
        )
        .robot_description_semantic(
            file_path="config/panda.srdf.xacro",
            mappings={
                "use_sensone_left": LaunchConfiguration("use_sensone_left"),
                "use_sensone_right": LaunchConfiguration("use_sensone_right"),
                "alter_finger_left": LaunchConfiguration("alter_finger_left"),
                "alter_finger_right": LaunchConfiguration("alter_finger_right"),
            },
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )
    ompl = _load_yaml("dual_arm_panda_moveit_config", "config/ompl_planning.yaml")
    chomp = _load_yaml("dual_arm_panda_moveit_config", "config/chomp_planning.yaml")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), ompl, chomp],
        condition=IfCondition(LaunchConfiguration("start_move_group")),
    )
    planner = Node(
        package="mtc_tutorial",
        executable="prior_single_arm_planner",
        output="screen",
        parameters=[moveit_config.to_dict(), ompl, chomp,
                    {"publish_legacy_topic": LaunchConfiguration("publish_legacy_topic"),
                     "alter_finger_left": LaunchConfiguration("alter_finger_left"),
                     "sync_via_move_group": LaunchConfiguration("sync_via_move_group")}],
    )
    subscriber = Node(
        package="mtc_tutorial",
        executable="prior_subtrajectory_subscriber",
        output="screen",
        parameters=[
            {"send_to_robot": LaunchConfiguration("send_to_robot")},
            {"follower_ip": LaunchConfiguration("follower_ip")},
            {"follower_port": LaunchConfiguration("follower_port")},
        ],
    )
    return LaunchDescription([
        use_sensone_left, use_sensone_right, alter_finger_left, alter_finger_right,
        send_to_robot, follower_ip, follower_port, publish_legacy_topic, start_move_group,
        sync_via_move_group,
        move_group, planner, subscriber,
    ])


def _load_yaml(package_name, file_path):
    import yaml
    path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)

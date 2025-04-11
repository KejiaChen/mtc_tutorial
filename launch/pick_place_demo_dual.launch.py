from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    exe_arg = DeclareLaunchArgument(name="exe")
    # scene_file_arg = DeclareLaunchArgument("scene_file",
    #                                         default_value="/home/tp2/ws_humble/scene/mongodb_8.scene",  # Default file path
    #                                         description="Path to the .scene file to be loaded",
    #                                     )
    
    # # Declare the path constraints argument
    # path_constraints_arg = DeclareLaunchArgument(
    #     "constraints",
    #     default_value="false",  # Default file path
    #     description="Whether to use path constraints",
    # )  

    use_sensone_left = DeclareLaunchArgument(
        "use_sensone_left",
        default_value="false",
        description="Whether to include the BotaSys external force torque sensor in the left robot model",
    )

    use_sensone_right = DeclareLaunchArgument(
        "use_sensone_right",
        default_value="true",
        description="Whether to include the BotaSys external force torque sensor in the right robot model",
    )
    
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_panda")
        .robot_description(file_path="config/panda.urdf.xacro",
                        #    mappings={
                        #         "ros2_control_hardware_type": LaunchConfiguration(
                        #             "ros2_control_hardware_type"
                        #         )
                        #     },
                           )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )


    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_tutorial",
        executable=LaunchConfiguration("exe"),
        # package="moveit2_tutorials",
        # executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sensone_left": LaunchConfiguration("use_sensone_left")},
            {"use_sensone_right": LaunchConfiguration("use_sensone_right")},
        ],
        # arguments=[
        #     # LaunchConfiguration("constraints")
        #     ],
    )
    
    # Distance Monitor node
    distance_monitor = Node(
        package="mtc_tutorial",
        executable="monitor_tracking",
        output="screen",
        # parameters=[
        #     moveit_config.to_dict(),
        # ],
    )
    
    # return LaunchDescription([exe_arg, scene_file_arg, pick_place_demo])
    return LaunchDescription([exe_arg, use_sensone_left, use_sensone_right, distance_monitor, pick_place_demo])

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():    
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
        .to_moveit_configs()
    )


    # DLO Demo node
    dlo_model = Node(
    package="mtc_tutorial",
    executable="dlo_line_model",
    output="screen",
    parameters=[
        moveit_config.to_dict(),
    ],
    )
    
    return LaunchDescription([dlo_model])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory("bot_description"), "urdf", "bot.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("bot", package_name="bot_moveit_config")
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path="config/bot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            # publish the planning scene + robot description for RViz
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
        ],
    )

    return LaunchDescription([move_group_node])

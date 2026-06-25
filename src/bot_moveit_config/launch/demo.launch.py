"""
RViz-only MoveIt demo (no Gazebo).

Brings up the robot with mock ros2_control hardware so MoveIt can plan AND
execute, with the result visualized in RViz. Near-instant RTF since there is
no physics simulation.

NOTE: the depth camera link is part of the robot model (visible in RViz) but
does NOT stream images here -- camera images require the Gazebo renderer.
Use `ros2 launch bot_gazebo sim.launch.xml` for live camera data.
"""
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
        .robot_description(file_path=urdf, mappings={"sim_mode": "mock"})
        .robot_description_semantic(file_path="config/bot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    pkg = get_package_share_directory("bot_moveit_config")
    ros2_controllers = os.path.join(pkg, "config", "ros2_controllers.yaml")
    rviz_config = os.path.join(pkg, "config", "moveit.rviz")

    use_sim_time = {"use_sim_time": False}

    # Publishes TF from the URDF + /joint_states
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, use_sim_time],
    )

    # controller_manager with mock hardware (reads robot_description from topic)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[ros2_controllers],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            jsb_spawner,
            arm_spawner,
            move_group,
            rviz,
        ]
    )

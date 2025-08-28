import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mobipick", package_name="mobipick_moveit2_config")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/mobipick.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    node = Node(
        package="grasplan_core",
        executable="mtc_pick.py",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"planning_scene_config_file": os.path.join(get_package_share_directory("grasplan_core"), "config", "cic_planning_scene.yaml")},
        ],
    )

    return LaunchDescription([node,])

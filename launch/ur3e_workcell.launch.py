#!/usr/bin/env python3
"""Launch the complete UR3e Gazebo Sim workcell, controllers, MoveIt and RViz."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_name = "ur3e_llm_control"
    package_share = get_package_share_directory(package_name)
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    moveit_config = (
        MoveItConfigsBuilder("ur3e_workcell", package_name=package_name)
        .robot_description(
            file_path="description/ur3e_with_gripper.urdf.xacro",
            mappings={"sim_gz": "true"},
        )
        .robot_description_semantic(file_path="config/ur3e_workcell.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    world_file = os.path.join(package_share, "description", "workcell.world")
    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": [" -r -v 3 ", world_file]}.items(),
        condition=IfCondition(gazebo_gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": [" -s -r --headless-rendering -v 3 ", world_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_ur3e",
        output="screen",
        arguments=[
            "-name", "ur3e_with_gripper",
            "-topic", "robot_description",
            "-allow_renaming", "true",
        ],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/ur3e_workcell/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/workcell_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/workcell_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
    )

    # Give Gazebo time to initialize, then activate the controllers in order.
    # A single spawner avoids concurrent controller-manager switch requests.
    controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "joint_trajectory_controller", "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "30",
        ],
        output="screen",
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[controllers],
        )
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.01,
                "trajectory_execution.execution_duration_monitoring": False,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", os.path.join(package_share, "RViz", "ur3e_workcell.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Read the sensor pose from the world so Gazebo and TF cannot drift apart.
    camera_model = ET.parse(world_file).find("./world/model[@name='workcell_camera']")
    camera_pose = camera_model.findtext("pose").split()
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="workcell_camera_tf",
        arguments=[
            "--x", camera_pose[0], "--y", camera_pose[1], "--z", camera_pose[2],
            "--roll", camera_pose[3], "--pitch", camera_pose[4], "--yaw", camera_pose[5],
            "--frame-id", "world", "--child-frame-id", "workcell_camera_link",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="workcell_camera_optical_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "-1.57079632679", "--pitch", "0", "--yaw", "-1.57079632679",
            "--frame-id", "workcell_camera_link",
            "--child-frame-id", "workcell_camera_optical_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            gazebo_with_gui,
            gazebo_headless,
            gz_bridge,
            robot_state_publisher,
            spawn_robot,
            start_controllers,
            camera_tf,
            camera_optical_tf,
            move_group,
            rviz,
        ]
    )

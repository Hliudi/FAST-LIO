#!/usr/bin/env python3
"""FAST-LIO2 LiDAR-inertial odometry in Gazebo (empty world), headless-capable.

Pipeline started by this launch file:

    Gazebo (empty world)
      + robot_fastlio.urdf  -> /lidar3d/points (PointCloud2) + /imu (Imu)
    pc2_to_livox.py bridge  -> /livox/lidar_front (livox_interfaces/CustomMsg)
    fastlio2_ros2 localizer -> /odom (nav_msgs/Odometry) + TF odom->base_link

Run (headless, no GUI):
    ros2 launch <repo>/launch/fastlio_sim.launch.py

Open RViz too:
    ros2 launch <repo>/launch/fastlio_sim.launch.py rviz:=true

Drive it (separate terminal, after sourcing the same ROS env):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
Then watch /odom and the odom->base_link TF track the motion.

Repo paths are resolved RELATIVE to this file (or $FASTLIO_SIM_REPO) so there
are no hardcoded absolute paths.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Repo root = parent of this launch/ directory. Override with FASTLIO_SIM_REPO.
REPO = os.environ.get(
    "FASTLIO_SIM_REPO",
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
)
URDF = os.path.join(REPO, "urdf", "robot_fastlio.urdf")
FLIO_CFG = os.path.join(REPO, "fast_lio_core", "config", "mid360_sim.yaml")
BRIDGE = os.path.join(REPO, "sim", "pc2_to_livox.py")
RVIZ = os.path.join(REPO, "rviz", "fastlio.rviz")


def generate_launch_description():
    gazebo_ros = get_package_share_directory("gazebo_ros")
    with open(URDF) as f:
        robot_desc = f.read()

    use_rviz = LaunchConfiguration("rviz")

    # gzserver only => headless. Empty world ships with gazebo_ros (no external
    # asset dependency). Set gui via gazebo.launch.py yourself if you want it.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(gazebo_ros, "worlds", "empty.world"),
            "verbose": "true",
            "pause": "false",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="rviz:=true also opens RViz with rviz/fastlio.rviz"),

        LogInfo(msg="[fastlio_sim] drive with:  "
                    "ros2 run teleop_twist_keyboard teleop_twist_keyboard"),

        gzserver,

        # Publishes the robot TF tree (base_link -> lidar_link) from the URDF.
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
             output="screen"),

        # Spawn once Gazebo is up.
        TimerAction(period=4.0, actions=[
            Node(package="gazebo_ros", executable="spawn_entity.py",
                 arguments=["-entity", "fastlio_bot",
                            "-topic", "robot_description",
                            "-z", "0.05"],
                 output="screen"),
        ]),

        # FAST-LIO pipeline: start after the robot (and its sensors) exist.
        TimerAction(period=7.0, actions=[
            # PointCloud2 -> Livox CustomMsg bridge (see README: why it exists).
            ExecuteProcess(cmd=["python3", BRIDGE], output="screen"),
            # The FAST-LIO2 localizer node.
            Node(package="fastlio2_ros2", executable="ros2_localizer",
                 parameters=[{
                     "lidar_topic": "/livox/lidar_front",
                     "imu_topic": "/imu",
                     "config_path": FLIO_CFG,
                     "frame_id": "odom",
                     "child_frame_id": "base_link",
                     "odom_topic": "/odom",
                     "use_sim_time": True,
                 }],
                 output="screen"),
        ]),

        # Optional RViz.
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", RVIZ],
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(use_rviz),
             output="screen"),
    ])

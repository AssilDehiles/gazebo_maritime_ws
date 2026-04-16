#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg        = get_package_share_directory("ros2_maritime")
    bridge_cfg = os.path.join(pkg, "config", "bridge_config.yaml")
    ekf_cfg    = os.path.join(pkg, "config", "ekf_filter.yaml")
    world_path = os.path.expanduser(
        "~/gazebo_maritime_ws/src/gazebo_maritime/worlds/monde_usv_leger.sdf"
    )
    return LaunchDescription([
        ExecuteProcess(cmd=["gz","sim","-r","-s",world_path], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="ros_gz_bridge", output="screen",
             parameters=[{"config_file": bridge_cfg}]),
        Node(package="ros2_maritime", executable="imu_relay",
             name="imu_relay", output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="odom_base_tf",
             arguments=["0","0","0","0","0","0","odom","base_link"]),
        Node(package="robot_localization", executable="ekf_node",
             name="ekf_filter_node_odom", output="screen",
             parameters=[ekf_cfg],
             remappings=[("/odometry/filtered","/odometry/local")]),
        Node(package="robot_localization", executable="ekf_node",
             name="ekf_filter_node_map", output="screen",
             parameters=[ekf_cfg]),
    ])

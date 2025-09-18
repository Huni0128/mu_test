#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    # ---- Args ----
    namespace    = LaunchConfiguration("namespace", default="")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    autostart    = LaunchConfiguration("autostart", default="true")
    log_level    = LaunchConfiguration("log_level", default="info")

    base_frame   = LaunchConfiguration("base_frame", default="base_link")
    odom_frame   = LaunchConfiguration("odom_frame", default="odom")
    map_frame    = LaunchConfiguration("map_frame",  default="map")
    laser_frame  = LaunchConfiguration("laser_frame", default="laser")
    publish_base_laser_tf = LaunchConfiguration("publish_base_laser_tf", default="false")

    # 기본 맵 경로: mu_cartographer/share/mu_cartographer/maps/map.yaml
    mu_cartographer_share = get_package_share_directory("mu_cartographer")
    default_map_yaml = os.path.join(mu_cartographer_share, "maps", "map.yaml")
    map_yaml    = LaunchConfiguration("map", default=default_map_yaml)

    # 파라미터 파일들
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    params_file_map_server = LaunchConfiguration(
        "params_file_map_server",
        default=os.path.join(nav2_bringup_share, "params", "nav2_params.yaml"),
    )
    mu_localization_share = get_package_share_directory("mu_localization")
    params_file_amcl = LaunchConfiguration(
        "params_file_amcl",
        default=os.path.join(mu_localization_share, "config", "amcl.yaml"),
    )

    decls = [
        DeclareLaunchArgument("namespace", default_value=namespace),
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("autostart", default_value=autostart),
        DeclareLaunchArgument("log_level", default_value=log_level),
        DeclareLaunchArgument("base_frame", default_value=base_frame),
        DeclareLaunchArgument("odom_frame", default_value=odom_frame),
        DeclareLaunchArgument("map_frame",  default_value=map_frame),
        DeclareLaunchArgument("laser_frame", default_value=laser_frame),
        DeclareLaunchArgument("publish_base_laser_tf", default_value=publish_base_laser_tf),
        DeclareLaunchArgument("map", default_value=default_map_yaml),
        DeclareLaunchArgument("params_file_map_server", default_value=params_file_map_server),
        DeclareLaunchArgument("params_file_amcl", default_value=params_file_amcl),
    ]

    # ---- Nodes (plain) ----
    ns_push = PushRosNamespace(namespace)

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file_map_server, {"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
        remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            params_file_amcl,
            {
                "use_sim_time": use_sim_time,
                "base_frame_id": base_frame,
                "odom_frame_id": odom_frame,
                "global_frame_id": map_frame,
            },
        ],
        # AMCL 입력 스캔을 /scan_cropped로 강제
        remappings=[("scan", "/scan_cropped"), ("tf", "/tf"), ("tf_static", "/tf_static")],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )

    lifecycle_mgr = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["map_server", "amcl"],
        }],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )

    static_tf = Node(
        condition=IfCondition(publish_base_laser_tf),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_laser_loc",
        # x y z qx qy qz qw parent child
        arguments=["0", "0", "0.20", "0", "0", "0", "1", base_frame, laser_frame],
        output="screen",
    )

    return LaunchDescription(decls + [ns_push, map_server, amcl, lifecycle_mgr, static_tf])

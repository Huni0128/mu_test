#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Cartographer online SLAM launch file.

- Frame/Topic/Config 값들을 Launch Argument로 노출
- Cartographer Node 및 Occupancy Grid Node 실행
- RViz2 자동 실행 (설정 파일 존재 시 -d 인자 추가)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    LaunchDescription 객체를 생성한다.

    Returns:
        LaunchDescription: Cartographer + occupancy grid + RViz2를
            포함한 launch 설정
    """
    namespace = LaunchConfiguration("namespace", default="")

    # Frames
    base_frame = LaunchConfiguration("base_frame", default="base_link")
    odom_frame = LaunchConfiguration("odom_frame", default="odom")
    map_frame = LaunchConfiguration("map_frame", default="map")

    # Topics
    scan_topic = LaunchConfiguration("scan_topic", default="/scan_cropped")

    # Cartographer
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    cfg_name = LaunchConfiguration(
        "configuration_basename",
        default="cartographer.lua",
    )

    # Occupancy Grid
    use_nav2 = LaunchConfiguration("use_nav2", default="false")
    occ_res = LaunchConfiguration("occ_resolution", default="0.05")
    occ_period = LaunchConfiguration("occ_publish_period_sec", default="1.0")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")

    pkg_share = get_package_share_directory("mu_cartographer")
    cfg_dir = os.path.join(pkg_share, "config")
    rviz_file = os.path.join(pkg_share, "rviz", "cartographer.rviz")
    rviz_args = ["-d", rviz_file] if os.path.exists(rviz_file) else []

    # ---- Arguments ----
    declared = [
        DeclareLaunchArgument("namespace", default_value=namespace),
        DeclareLaunchArgument("base_frame", default_value=base_frame),
        DeclareLaunchArgument("odom_frame", default_value=odom_frame),
        DeclareLaunchArgument("map_frame", default_value=map_frame),
        DeclareLaunchArgument("scan_topic", default_value=scan_topic),
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument(
            "configuration_basename",
            default_value=cfg_name,
        ),
        DeclareLaunchArgument("use_nav2", default_value=use_nav2),
        DeclareLaunchArgument("occ_resolution", default_value=occ_res),
        DeclareLaunchArgument(
            "occ_publish_period_sec",
            default_value=occ_period,
        ),
        DeclareLaunchArgument("launch_rviz", default_value=launch_rviz),
    ]

    # ---- Nodes ----
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        namespace=namespace,
        output="screen",
        arguments=[
            "-configuration_directory",
            cfg_dir,
            "-configuration_basename",
            cfg_name,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("scan", scan_topic)],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="occupancy_grid_node",
        name="occupancy_grid_node",
        namespace=namespace,
        output="screen",
        arguments=[
            "-resolution",
            occ_res,
            "-publish_period_sec",
            occ_period,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_nav2),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        arguments=rviz_args,
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        declared + [cartographer_node, occupancy_grid_node, rviz_node]
    )

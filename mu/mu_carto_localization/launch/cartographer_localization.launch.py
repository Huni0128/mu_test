#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Cartographer localization launch for ROS 2 (Foxy/py3.8 compatible).

- pbstream/map.yaml 자동 탐색(설치경로 우선, 그다음 소스 상대경로)
- Cartographer 약간 지연 시작(tf_static 먼저 수신)
- 선택적 map_server + lifecycle + RViz2
- 필요 시 base_link -> laser 정적 TF 발행
"""

import os
from glob import glob
from typing import Optional, List  # <-- py3.8 호환

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def _find_maps_dir() -> Optional[str]:
    """지도 디렉터리 경로를 탐색해 반환한다.

    우선순위:
    1) 설치 패키지(mu_cartographer) share/maps
    2) 소스 상대경로 후보들(현재 파일 기준, CWD 기준)
    """
    # 1) install 경로 우선
    try:
        share = get_package_share_directory("mu_cartographer")
        maps_dir = os.path.join(share, "maps")
        if os.path.isdir(maps_dir):
            return maps_dir
    except Exception:
        pass

    # 2) 소스 상대경로 후보
    base_dir = os.path.dirname(__file__)  # 이 launch 파일 위치
    candidates = [
        os.path.join(base_dir, "..", "mu", "mu_cartographer", "maps"),
        os.path.join(os.getcwd(), "mu", "mu_cartographer", "maps"),
    ]
    for path in candidates:
        abs_path = os.path.abspath(path)
        if os.path.isdir(abs_path):
            return abs_path

    return None


def _pick_first(patterns: List[str]) -> Optional[str]:
    """패턴 리스트에서 첫 번째로 매칭되는 파일 하나를 반환한다."""
    for pat in patterns:
        files = sorted(glob(pat))
        if files:
            return files[0]
    return None


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("mu_carto_localization")
    config_dir = os.path.join(pkg_share, "config")
    rviz_default = os.path.join(pkg_share, "rviz", "mu_carto_loc.rviz")

    # 지도 기본 경로 자동 탐색
    maps_dir = _find_maps_dir()
    default_pbstream = None
    default_map_yaml = None
    if maps_dir:
        preferred_pb = os.path.join(maps_dir, "map.pbstream")
        default_pbstream = (
            preferred_pb
            if os.path.isfile(preferred_pb)
            else _pick_first([os.path.join(maps_dir, "*.pbstream")])
        )
        preferred_yaml = os.path.join(maps_dir, "map.yaml")
        default_map_yaml = (
            preferred_yaml
            if os.path.isfile(preferred_yaml)
            else _pick_first([os.path.join(maps_dir, "*.yaml")])
        )

    # ---- Args ----
    namespace = LaunchConfiguration("namespace", default="")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    log_level = LaunchConfiguration("log_level", default="info")
    scan_topic = LaunchConfiguration("scan_topic", default="/scan_cropped")

    load_state = LaunchConfiguration(
        "load_state_filename",
        default=TextSubstitution(text=default_pbstream or ""),
    )
    map_yaml = LaunchConfiguration(
        "map",
        default=TextSubstitution(
            text=(
                default_map_yaml
                or os.path.join(
                    get_package_share_directory("mu_cartographer"),
                    "maps",
                    "map.yaml",
                )
            )
        ),
    )

    map_frame = LaunchConfiguration("map_frame", default="map")
    odom_frame = LaunchConfiguration("odom_frame", default="odom")
    base_frame = LaunchConfiguration("base_frame", default="base_link")
    laser_frame = LaunchConfiguration("laser_frame", default="laser")

    publish_base_laser_tf = LaunchConfiguration(
        "publish_base_laser_tf", default="false"
    )
    use_map_server = LaunchConfiguration("use_map_server", default="true")
    enable_rviz = LaunchConfiguration("enable_rviz", default="true")
    rviz_config = LaunchConfiguration("rviz_config", default=rviz_default)
    carto_start_delay = LaunchConfiguration("carto_start_delay", default="0.3")  # 초

    declared = [
        DeclareLaunchArgument("namespace", default_value=namespace),
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("log_level", default_value=log_level),
        DeclareLaunchArgument("scan_topic", default_value=scan_topic),
        DeclareLaunchArgument(
            "load_state_filename",
            default_value=load_state,
            description=".pbstream (자동 탐색 기본값 사용)",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=map_yaml,
            description="map.yaml (자동 탐색 기본값 사용)",
        ),
        DeclareLaunchArgument("map_frame", default_value=map_frame),
        DeclareLaunchArgument("odom_frame", default_value=odom_frame),
        DeclareLaunchArgument("base_frame", default_value=base_frame),
        DeclareLaunchArgument("laser_frame", default_value=laser_frame),
        DeclareLaunchArgument(
            "publish_base_laser_tf", default_value=publish_base_laser_tf
        ),
        DeclareLaunchArgument("use_map_server", default_value=use_map_server),
        DeclareLaunchArgument("enable_rviz", default_value=enable_rviz),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config),
        DeclareLaunchArgument("carto_start_delay", default_value=carto_start_delay),
    ]

    def setup(ctx, *_args, **_kwargs):
        load_path = load_state.perform(ctx)
        yaml_path = map_yaml.perform(ctx)

        nodes = []

        # 안내 로그
        if not load_path:
            msg = (
                "[mu_carto_localization] WARNING: .pbstream을 찾지 못했습니다. "
                "load_state_filename 인자로 지정하세요."
            )
            if maps_dir:
                msg += f" (탐색 디렉토리: {maps_dir})"
            nodes.append(LogInfo(msg=msg))
        else:
            nodes.append(
                LogInfo(msg=f"[mu_carto_localization] Using pbstream: {load_path}")
            )
        if yaml_path:
            nodes.append(
                LogInfo(msg=f"[mu_carto_localization] Using map.yaml: {yaml_path}")
            )

        # Cartographer (약간 지연 시작: /tf_static 먼저 수신하도록)
        carto_args = [
            "-configuration_directory",
            config_dir,
            "-configuration_basename",
            "cartographer_2d_localization.lua",
        ]
        if load_path:
            carto_args += ["-load_state_filename", load_path]

        carto_node = Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            namespace=namespace,
            output="screen",
            arguments=carto_args,
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[("scan", scan_topic)],
            emulate_tty=True,
        )
        delayed_carto = TimerAction(
            period=float(carto_start_delay.perform(ctx)),
            actions=[carto_node],
        )

        # map_server + lifecycle
        map_server_node = Node(
            condition=IfCondition(use_map_server),
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
            remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
            arguments=["--ros-args", "--log-level", log_level],
            emulate_tty=True,
        )
        lifecycle_mgr = Node(
            condition=IfCondition(use_map_server),
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "node_names": ["map_server"],
                }
            ],
            arguments=["--ros-args", "--log-level", log_level],
            emulate_tty=True,
        )

        # 필요 시 정적 TF(base_link -> laser) 발행
        static_tf = Node(
            condition=IfCondition(publish_base_laser_tf),
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_to_laser_carto",
            arguments=["0", "0", "0.20", "0", "0", "0", "1", base_frame, laser_frame],
            output="screen",
        )

        # RViz2 자동 실행
        rviz_node = Node(
            condition=IfCondition(enable_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            emulate_tty=True,
        )

        nodes += [static_tf, map_server_node, lifecycle_mgr, delayed_carto, rviz_node]
        return nodes

    return LaunchDescription(declared + [OpaqueFunction(function=setup)])

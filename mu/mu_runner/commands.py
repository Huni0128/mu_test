"""Utilities for building ROS 2 launch commands."""
from __future__ import annotations

from typing import List, Optional

from .config import MuConfig
from .env import build_source_prefix


def build_launch_command(
    cfg: MuConfig,
    package: str,
    launch_file: str,
    extra_ros_args: Optional[List[str]] = None,
) -> str:
    """Return the ros2 launch command string that should be executed."""
    prefix = build_source_prefix(cfg)
    args_str = " ".join(extra_ros_args) if extra_ros_args else ""
    base = f"{prefix} && ros2 launch {package} {launch_file}"
    return f"{base} {args_str}".strip()


def build_shell_command(cfg: MuConfig, command: str) -> List[str]:
    """Wrap a command string with the proper bash invocation."""
    shell = ["bash", "-c", command]
    if cfg.interactive_bash:
        shell.insert(1, "-i")
    return shell
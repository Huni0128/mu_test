"""Backward-compatible launch helpers that wrap :mod:`mu.mu_runner.process`."""
from __future__ import annotations

import subprocess
from typing import List, Optional

from .commands import build_launch_command, build_shell_command
from .config import MuConfig
from .process import (
    RosLaunchProcess,
    ensure_ros_setup,
    stop_process_tree,
)


def build_launch_cmd(cfg: MuConfig, extra_ros_args: Optional[List[str]] = None) -> str:
    """Return the ros2 launch command for the configured package."""
    return build_launch_command(
        cfg,
        cfg.package,
        cfg.launch_file,
        extra_ros_args=extra_ros_args,
    )

def launch(cfg: MuConfig, extra_ros_args: Optional[List[str]] = None) -> None:
    if not cfg.ros_setup.exists():
        """Run ros2 launch synchronously using :func:`subprocess.run`."""
    ensure_ros_setup(cfg)
    cmd = build_launch_cmd(cfg, extra_ros_args)
    shell = build_shell_command(cfg, cmd)
    print(f"[INFO] Executing (sync): {' '.join(shell)}")
    subprocess.run(shell, check=True)

def launch_async(
    cfg: MuConfig, extra_ros_args: Optional[List[str]] = None
) -> subprocess.Popen:
    """Launch ros2 in the background and return the :class:`Popen` handle."""
    launcher = RosLaunchProcess(
        cfg=cfg,
        package=cfg.package,
        launch_file=cfg.launch_file,
        extra_ros_args=extra_ros_args,
    )
    ensure_ros_setup(cfg)
    cmd = build_launch_cmd(cfg, extra_ros_args)
    shell = build_shell_command(cfg, cmd)
    print(f"[INFO] Executing (async): {' '.join(shell)}")
    return launcher.start()


__all__ = [
    "build_launch_cmd",
    "launch",
    "launch_async",
    "stop_process_tree",
    "RosLaunchProcess",
]
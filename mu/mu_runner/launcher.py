"""Backward-compatible launch helpers that wrap :mod:`mu.mu_runner.process`."""
from __future__ import annotations

import subprocess
from typing import List, Optional

from .config import MuConfig
from .process import (
    RosLaunchProcess,
    create_launch_request,
    ensure_ros_setup,
    stop_process_tree,
)


def build_launch_cmd(cfg: MuConfig, extra_ros_args: Optional[List[str]] = None) -> str:
    """Return the ros2 launch command for the configured package."""
    request = create_launch_request(
        cfg,
        cfg.package,
        cfg.launch_file,
        extra_ros_args=extra_ros_args,
    )
    return request.command()

def launch(cfg: MuConfig, extra_ros_args: Optional[List[str]] = None) -> None:
    """Run ros2 launch synchronously using :func:`subprocess.run`."""
    request = create_launch_request(
        cfg,
        cfg.package,
        cfg.launch_file,
        extra_ros_args=extra_ros_args,
    )
    ensure_ros_setup(cfg)
    shell = request.shell()
    _log_shell("sync", shell)
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
    shell = launcher.request.shell()
    _log_shell("async", shell)
    return launcher.start()


__all__ = [
    "build_launch_cmd",
    "launch",
    "launch_async",
    "stop_process_tree",
    "RosLaunchProcess",
]


def _log_shell(mode: str, shell: List[str]) -> None:
    print(f"[INFO] Executing ({mode}): {' '.join(shell)}")
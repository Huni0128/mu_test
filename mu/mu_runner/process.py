"""Process helpers for managing ros2 launch invocations."""
from __future__ import annotations

import os
import signal
import subprocess
import threading
from dataclasses import dataclass, field
from typing import Iterable, List, Optional

from .config import MuConfig
from .commands import build_launch_command, build_shell_command


class RosSetupNotFoundError(FileNotFoundError):
    """Raised when the configured ROS setup.bash cannot be found."""


def ensure_ros_setup(cfg: MuConfig) -> None:
    if not cfg.ros_setup.exists():
        raise RosSetupNotFoundError(f"ROS setup not found: {cfg.ros_setup}")


@dataclass
class LaunchRequest:
    cfg: MuConfig
    package: str
    launch_file: str
    extra_ros_args: List[str] = field(default_factory=list)

    def command(self) -> str:
        return build_launch_command(
            self.cfg,
            self.package,
            self.launch_file,
            self.extra_ros_args or None,
        )

    def shell(self) -> List[str]:
        return build_shell_command(self.cfg, self.command())

def create_launch_request(
    cfg: MuConfig,
    package: str,
    launch_file: str,
    extra_ros_args: Optional[Iterable[str]] = None,
) -> LaunchRequest:
    return LaunchRequest(
        cfg=cfg,
        package=package,
        launch_file=launch_file,
        extra_ros_args=list(extra_ros_args or []),
    )


class RosLaunchProcess:
    """Thread-safe wrapper around ``subprocess.Popen`` for ros2 launch."""

    def __init__(
        self,
        cfg: MuConfig,
        package: str,
        launch_file: str,
        extra_ros_args: Optional[Iterable[str]] = None,
    ) -> None:
        self.request = create_launch_request(
            cfg=cfg,
            package=package,
            launch_file=launch_file,
            extra_ros_args=extra_ros_args,
        )
        self._lock = threading.RLock()
        self._proc: Optional[subprocess.Popen] = None

    def update_extra_args(self, extra_ros_args: Optional[Iterable[str]]) -> None:
        with self._lock:
            self.request.extra_ros_args = list(extra_ros_args or [])

    # Public API ---------------------------------------------------------
    def start(self) -> subprocess.Popen:
        """Start ros2 launch in the background if it is not already running."""
        with self._lock:
            if self._proc and self._proc.poll() is None:
                return self._proc
            ensure_ros_setup(self.request.cfg)
            shell = self.request.shell()
            self._proc = subprocess.Popen(shell, preexec_fn=os.setsid)
            return self._proc

    def run_sync(self) -> None:
        """Run the launch process and block until completion."""
        proc = self.start()
        proc.wait()

    def stop(self, sig: signal.Signals = signal.SIGINT, wait_seconds: float = 8.0) -> None:
        """Terminate the ros2 launch process tree if it is running."""
        proc = self._acquire_process()
        if proc is None:
            return
        try:
            stop_process_tree(proc, sig=sig, wait_seconds=wait_seconds)
        finally:
            with self._lock:
                self._proc = None

    def is_running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def process(self) -> Optional[subprocess.Popen]:
        with self._lock:
            return self._proc

    # Internal helpers --------------------------------------------------
    def _acquire_process(self) -> Optional[subprocess.Popen]:
        with self._lock:
            return self._proc


def stop_process_tree(
    proc: subprocess.Popen,
    sig: signal.Signals = signal.SIGINT,
    wait_seconds: float = 8.0,
) -> None:
    """Send ``sig`` to the process group and wait for termination."""
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, sig)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass
    try:
        proc.wait(timeout=wait_seconds)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass
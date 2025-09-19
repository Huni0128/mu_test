"""Qt wrappers around :mod:`mu.mu_runner.process` utilities."""
from __future__ import annotations

from typing import Iterable, Optional

from PyQt5.QtCore import QObject, pyqtSignal

from mu.mu_runner.config import MuConfig
from mu.mu_runner.process import RosLaunchProcess


class LaunchProcessManager(QObject):
    state_changed = pyqtSignal(bool, str)
    error_signal = pyqtSignal(str)

    def __init__(
        self,
        cfg: MuConfig,
        package: str,
        launch_file: str,
        label: str,
        extra_args: Optional[Iterable[str]] = None,
    ) -> None:
        super().__init__()
        self.label = label
        self._launcher = RosLaunchProcess(
            cfg=cfg,
            package=package,
            launch_file=launch_file,
            extra_ros_args=extra_args,
        )

    def update_extra_args(self, args: Optional[Iterable[str]]) -> None:
        self._launcher.update_extra_args(args)

    def start(self) -> None:
        if self._launcher.is_running():
            self.state_changed.emit(True, f"{self.label}: running")
            return
        try:
            self._launcher.start()
        except Exception as exc:  # pragma: no cover - subprocess errors
            self.error_signal.emit(f"Failed to start {self.label.lower()}: {exc}")
            return
        self.state_changed.emit(True, f"{self.label}: running")

    def stop(self, wait_seconds: float = 8.0) -> None:
        if not self._launcher.is_running():
            self.state_changed.emit(False, f"{self.label}: stopped")
            return
        try:
            self._launcher.stop(wait_seconds=wait_seconds)
        except Exception as exc:  # pragma: no cover - subprocess errors
            self.error_signal.emit(f"Failed to stop {self.label.lower()}: {exc}")
        finally:
            self.state_changed.emit(False, f"{self.label}: stopped")

    def is_running(self) -> bool:
        return self._launcher.is_running()


class BringupManager(LaunchProcessManager):
    def __init__(self, cfg: MuConfig) -> None:
        super().__init__(cfg, cfg.package, cfg.launch_file, "Bringup")


class CartographerManager(LaunchProcessManager):
    def __init__(self, cfg: MuConfig) -> None:
        super().__init__(
            cfg,
            cfg.cartographer_package,
            cfg.cartographer_launch_file,
            "Cartographer",
            extra_args=["launch_rviz:=false"],
        )
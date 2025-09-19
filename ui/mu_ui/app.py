"""Application entry-points for the MU PyQt5 GUI."""
from __future__ import annotations

import sys
from typing import Any

from PyQt5.QtWidgets import QApplication

from mu.mu_runner.config import MuConfig

from .main_window import MuMainWindow


def run_app(cfg: MuConfig) -> None:
    """Create the QApplication and display the main window."""
    app = QApplication(sys.argv)
    win = MuMainWindow(cfg)
    win.show()
    sys.exit(app.exec_())


def _default_config() -> MuConfig:
    """Construct a :class:`MuConfig` instance with default values."""
    # The original single-file script fabricated an argparse Namespace with
    # default values when executed directly. We mimic the same behaviour here
    # for backwards compatibility.
    attrs: dict[str, Any] = {
        "ros_distro": None,
        "ws_root": None,
        "package": None,
        "launch_file": None,
        "no_interactive": False,
        "carto_package": None,
        "carto_launch": None,
        "map_topic": None,
        "robot_frame": None,
    }
    namespace = type("Args", (), attrs)()
    return MuConfig.from_args(namespace)  # type: ignore[arg-type]


if __name__ == "__main__":  # pragma: no cover - manual testing helper
    run_app(_default_config())
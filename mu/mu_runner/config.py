from dataclasses import dataclass
from pathlib import Path
import os
import argparse

@dataclass
class MuConfig:
    ros_distro: str
    ws_root: Path
    package: str
    launch_file: str
    interactive_bash: bool = True

    @property
    def ros_setup(self) -> Path:
        return Path(f"/opt/ros/{self.ros_distro}/setup.bash")

    @property
    def ws_setup(self) -> Path:
        return self.ws_root / "install" / "setup.bash"

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> "MuConfig":
        # 프로젝트 루트 = main_mu.py가 있는 폴더라고 가정
        project_root = Path(__file__).resolve().parents[2]
        ws_root = Path(args.ws_root).resolve() if args.ws_root else project_root
        return cls(
            ros_distro=args.ros_distro or os.environ.get("ROS_DISTRO", "foxy"),
            ws_root=ws_root,
            package=args.package or "mu_bringup",
            launch_file=args.launch_file or "bringup.launch.py",
            interactive_bash=not args.no_interactive,
        )

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run a ROS 2 launch with modular workflow.")
    p.add_argument("--ros-distro", help="ROS distro (e.g., foxy, humble)")
    p.add_argument("--ws-root", help="Workspace root (default: project root)")
    p.add_argument("--package", help="ROS 2 package name (default: mu_bringup)")
    p.add_argument("--launch-file", help="Launch file (default: bringup.launch.py)")
    p.add_argument("--no-interactive", action="store_true",
                   help="Do not use interactive bash (-i).")
    return p.parse_args()

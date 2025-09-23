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
    cartographer_package: str = "mu_cartographer"
    cartographer_launch_file: str = "cartographer.launch.py"
    map_topic: str = "/map"
    robot_frame: str = "base_link"

    @property
    def ros_setup(self) -> Path:
        return Path(f"/opt/ros/{self.ros_distro}/setup.bash")

    @property
    def ws_setup(self) -> Path:
        return self.ws_root / "install" / "setup.bash"

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> "MuConfig":
        project_root = Path(__file__).resolve().parents[2]
        ws_root = Path(args.ws_root).resolve() if args.ws_root else project_root
        return cls(
            ros_distro=args.ros_distro or os.environ.get("ROS_DISTRO", "foxy"),
            ws_root=ws_root,
            package=args.package or "mu_bringup",
            launch_file=args.launch_file or "bringup.launch.py",
            interactive_bash=not args.no_interactive,
            cartographer_package=args.carto_package or "mu_cartographer",
            cartographer_launch_file=args.carto_launch or "cartographer.launch.py",
            map_topic=args.map_topic or "/map",
            robot_frame=args.robot_frame or "base_link",
        )

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run a ROS 2 launch with modular workflow.")
    p.add_argument("--ros-distro", help="ROS distro (e.g., foxy, humble)")
    p.add_argument("--ws-root", help="Workspace root (default: project root)")
    p.add_argument("--package", help="ROS 2 package name (default: mu_bringup)")
    p.add_argument("--launch-file", help="Launch file (default: bringup.launch.py)")
    p.add_argument("--no-interactive", action="store_true",
                   help="Do not use interactive bash (-i).")
    p.add_argument("--carto-package", dest="carto_package",
                   help="Cartographer package name (default: mu_cartographer)")
    p.add_argument("--carto-launch", dest="carto_launch",
                   help="Cartographer launch file (default: cartographer.launch.py)")
    p.add_argument("--map-topic", dest="map_topic",
                   help="Occupancy grid topic to visualize (default: /map)")
    p.add_argument("--robot-frame", dest="robot_frame",
                   help="TF frame name for the robot base (default: base_link)")
    return p.parse_args()

#!/usr/bin/env python3
from mu.mu_runner.config import MuConfig, parse_args
from ui.mu_ui import run_app  # GUI 진입점

def main():
    cfg = MuConfig.from_args(parse_args())
    print(f"[INFO] Launching GUI (ROS_DISTRO={cfg.ros_distro}, WS_ROOT={cfg.ws_root})")
    run_app(cfg)

if __name__ == "__main__":
    main()

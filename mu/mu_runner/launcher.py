# mu/mu_runner/launcher.py
import os
import signal
import subprocess
from typing import List, Optional
from .config import MuConfig
from .env import build_source_prefix

def build_launch_cmd(cfg: MuConfig, extra_ros_args: Optional[List[str]] = None) -> str:
    prefix = build_source_prefix(cfg)
    args_str = " ".join(extra_ros_args) if extra_ros_args else ""
    return f"{prefix} && ros2 launch {cfg.package} {cfg.launch_file} {args_str}".strip()

def launch(cfg: MuConfig, extra_ros_args: Optional[List[str]] = None) -> None:
    if not cfg.ros_setup.exists():
        raise FileNotFoundError(f"ROS setup not found: {cfg.ros_setup}")
    cmd = build_launch_cmd(cfg, extra_ros_args)
    shell = ["bash", "-i", "-c", cmd] if cfg.interactive_bash else ["bash", "-c", cmd]
    print(f"[INFO] Executing (sync): {' '.join(shell)}")
    subprocess.run(shell, check=True)

def launch_async(cfg: MuConfig, extra_ros_args: Optional[List[str]] = None) -> subprocess.Popen:
    """ros2 launch를 백그라운드로 실행하고 Popen을 반환."""
    if not cfg.ros_setup.exists():
        raise FileNotFoundError(f"ROS setup not found: {cfg.ros_setup}")
    cmd = build_launch_cmd(cfg, extra_ros_args)
    shell = ["bash", "-i", "-c", cmd] if cfg.interactive_bash else ["bash", "-c", cmd]
    print(f"[INFO] Executing (async): {' '.join(shell)}")
    # 새 프로세스 그룹으로 실행 → 나중에 그룹 전체에 SIGINT/SIGTERM 보낼 수 있음
    return subprocess.Popen(shell, preexec_fn=os.setsid)

def stop_process_tree(proc: subprocess.Popen, sig=signal.SIGINT, wait_seconds: float = 8.0):
    """ros2 launch 프로세스 그룹에 신호를 보내서 정리."""
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, sig)
    except Exception:
        # 마지막 수단으로 terminate/kill
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

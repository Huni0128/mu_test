from .config import MuConfig
import shlex

def build_source_prefix(cfg: MuConfig) -> str:
    parts = [f"source {shlex.quote(str(cfg.ros_setup))}"]
    if cfg.ws_setup.exists():
        parts.append(f"source {shlex.quote(str(cfg.ws_setup))}")
    return " && ".join(parts)

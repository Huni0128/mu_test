"""Utility helpers related to occupancy grid maps."""
from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path

from nav_msgs.msg import OccupancyGrid


def format_map_info(grid: OccupancyGrid) -> str:
    resolution = grid.info.resolution
    width = grid.info.width
    height = grid.info.height
    return f"Resolution: {resolution:.3f} m | Size: {width} x {height}"


def format_stamp(stamp) -> str:
    if stamp is None:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    secs = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if secs is None or nanosec is None:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    if secs <= 0:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        dt = datetime.fromtimestamp(secs + nanosec / 1e9)
        return dt.strftime("%Y-%m-%d %H:%M:%S")
    except (OverflowError, OSError, ValueError):
        return f"{secs}.{int(nanosec):09d}"


def write_map_files(pgm_path: Path, grid: OccupancyGrid) -> Path:
    width = int(grid.info.width)
    height = int(grid.info.height)
    if width <= 0 or height <= 0:
        raise ValueError("Map has zero dimensions")
    data = grid.data
    if len(data) != width * height:
        raise ValueError("Map data length mismatch")
    pixels = bytearray(width * height)
    for idx, occ in enumerate(data):
        pixels[idx] = occupancy_to_gray(occ)
    flipped = bytearray(width * height)
    for y in range(height):
        src = (height - 1 - y) * width
        dst = y * width
        flipped[dst : dst + width] = pixels[src : src + width]
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    with open(pgm_path, "wb") as f:
        f.write(header)
        f.write(flipped)
    yaml_path = pgm_path.with_suffix(".yaml")
    yaw = quaternion_to_yaw(grid.info.origin.orientation)
    yaml_content = (
        f"image: {pgm_path.name}\n"
        f"mode: trinary\n"
        f"resolution: {grid.info.resolution}\n"
        f"origin: [{grid.info.origin.position.x}, {grid.info.origin.position.y}, {yaw}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)
    return yaml_path


def occupancy_to_gray(occ: int) -> int:
    if occ < 0:
        return 205
    return max(0, min(255, int(round(255 - (occ / 100.0) * 255))))


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
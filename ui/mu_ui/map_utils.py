"""Utility helpers related to occupancy grid maps."""
from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Sequence, Tuple

from nav_msgs.msg import OccupancyGrid


@dataclass(frozen=True)
class OccupancyGridImage:
    """Simple container for a grayscale occupancy grid image."""

    width: int
    height: int
    pixels: bytearray


def format_map_info(grid: OccupancyGrid) -> str:
    resolution = grid.info.resolution
    width = grid.info.width
    height = grid.info.height
    return f"Resolution: {resolution:.3f} m | Size: {width} x {height}"


def format_stamp(stamp) -> str:
    if stamp is None:
        return _now_str()
    secs = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if secs is None or nanosec is None or secs <= 0:
        return _now_str()
    try:
        dt = datetime.fromtimestamp(secs + nanosec / 1e9)
        return dt.strftime("%Y-%m-%d %H:%M:%S")
    except (OverflowError, OSError, ValueError):
        return f"{secs}.{int(nanosec):09d}"


def write_map_files(pgm_path: Path, grid: OccupancyGrid) -> Path:
    image = occupancy_grid_to_image(grid)
    header = f"P5\n{image.width} {image.height}\n255\n".encode("ascii")
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    with open(pgm_path, "wb") as f:
        f.write(header)
        f.write(image.pixels)
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


def occupancy_grid_to_image(grid: OccupancyGrid) -> OccupancyGridImage:
    width, height = _grid_size(grid)
    pixels = _grid_to_grayscale_bytes(grid.data, width, height)
    flipped = _flip_vertical(pixels, width, height)
    return OccupancyGridImage(width=width, height=height, pixels=flipped)


def occupancy_to_gray(occ: int) -> int:
    if occ < 0:
        return 205
    return max(0, min(255, int(round(255 - (occ / 100.0) * 255))))


def quaternion_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _now_str() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def _grid_size(grid: OccupancyGrid) -> Tuple[int, int]:
    width = int(grid.info.width)
    height = int(grid.info.height)
    if width <= 0 or height <= 0:
        raise ValueError("Map has zero dimensions")
    return width, height


def _grid_to_grayscale_bytes(data: Sequence[int], width: int, height: int) -> bytearray:
    expected = width * height
    if len(data) != expected:
        raise ValueError("Map data length mismatch")
    pixels = bytearray(expected)
    for idx, occ in enumerate(data):
        pixels[idx] = occupancy_to_gray(int(occ))
    return pixels


def _flip_vertical(buffer: Sequence[int], width: int, height: int) -> bytearray:
    flipped = bytearray(width * height)
    row_span = width
    for y in range(height):
        src = (height - 1 - y) * row_span
        dst = y * row_span
        segment = buffer[src : src + row_span]
        if isinstance(segment, (bytes, bytearray)):
            flipped[dst : dst + row_span] = segment
        else:
            for offset, value in enumerate(segment):
                flipped[dst + offset] = int(value)
    return flipped
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PyQt5 GUI for:
 - Bringup start/stop (ros2 launch mu_bringup bringup.launch.py)
 - Browse ROS 2 topics and echo latest message as YAML
Python 3.8 + ROS 2 Foxy 호환
"""
import sys
import os
import signal
import subprocess
import threading
import traceback
import math
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Tuple, Callable

from PyQt5 import uic
from PyQt5.QtCore import QObject, pyqtSignal, Qt, QTimer, QRectF
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QMessageBox,
    QListWidgetItem,
    QFileDialog,
    QGraphicsScene,
    QGraphicsView,
)
from PyQt5.QtGui import QImage, QPixmap, QPen, QBrush, QColor, QPainter

# ROS 2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_yaml
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

# 기존 설정/환경 유틸 재사용
from mu.mu_runner.config import MuConfig
from mu.mu_runner.env import build_source_prefix


# -------- Launch process managers --------
class LaunchProcessManager(QObject):
    state_changed = pyqtSignal(bool, str)
    error_signal = pyqtSignal(str)

    def __init__(
        self,
        cfg: MuConfig,
        package: str,
        launch_file: str,
        label: str,
        extra_args: Optional[List[str]] = None,
    ):
        super().__init__()
        self.cfg = cfg
        self.package = package
        self.launch_file = launch_file
        self.label = label
        self._extra_args = extra_args or []
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.RLock()

    def update_extra_args(self, args: Optional[List[str]]):
        with self._lock:
            self._extra_args = args or []

    def _build_cmd(self) -> List[str]:
        prefix = build_source_prefix(self.cfg)
        args_str = " ".join(self._extra_args) if self._extra_args else ""
        cmd = f"{prefix} && ros2 launch {self.package} {self.launch_file}"
        if args_str:
            cmd = f"{cmd} {args_str}"
        shell = ["bash", "-i", "-c", cmd] if self.cfg.interactive_bash else ["bash", "-c", cmd]
        return shell

    def start(self):
        with self._lock:
            if self._proc and self._proc.poll() is None:
                self.state_changed.emit(True, f"{self.label}: running")
                return
            try:
                shell = self._build_cmd()
                self._proc = subprocess.Popen(shell, preexec_fn=os.setsid)
                self.state_changed.emit(True, "Bringup: running")
            except Exception as e:
                self._proc = None
                self.error_signal.emit(f"Failed to start bringup: {e}")

    def stop(self, wait_seconds: float = 8.0):
        with self._lock:
            if not self._proc:
                self.state_changed.emit(False, f"{self.label}: stopped")
                return
            try:
                pgid = os.getpgid(self._proc.pid)
                os.killpg(pgid, signal.SIGINT)
            except Exception:
                try:
                    self._proc.terminate()
                except Exception:
                    pass
            try:
                self._proc.wait(timeout=wait_seconds)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
            finally:
                self._proc = None
                self.state_changed.emit(False, f"{self.label}: stopped")

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None


class BringupManager(LaunchProcessManager):
    def __init__(self, cfg: MuConfig):
        super().__init__(cfg, cfg.package, cfg.launch_file, "Bringup")


class CartographerManager(LaunchProcessManager):
    def __init__(self, cfg: MuConfig):
        extra = ["launch_rviz:=false"]
        super().__init__(
            cfg,
            cfg.cartographer_package,
            cfg.cartographer_launch_file,
            "Cartographer",
            extra_args=extra,
        )


class MapSignal(QObject):
    map_received = pyqtSignal(object)


class MapGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setRenderHint(QPainter.SmoothPixmapTransform, True)
        self._map_item = None
        self._auto_fit = True
        self._zoom = 0
        self._zoom_step = 1.25
        self._zoom_range = (-20, 50)

    def update_map_item(self, item, size_changed: bool):
        self._map_item = item
        if self._map_item is None:
            self.resetTransform()
            self._zoom = 0
            return
        if size_changed:
            self.reset_view()
        elif self._auto_fit:
            self._apply_fit()

    def reset_view(self):
        self._auto_fit = True
        self._zoom = 0
        self._apply_fit()

    def wheelEvent(self, event):
        if self._map_item is None:
            event.ignore()
            return
        delta = event.angleDelta().y()
        if delta == 0:
            event.ignore()
            return
        direction = 1 if delta > 0 else -1
        new_zoom = self._zoom + direction
        if new_zoom < self._zoom_range[0] or new_zoom > self._zoom_range[1]:
            event.ignore()
            return
        factor = self._zoom_step if direction > 0 else 1.0 / self._zoom_step
        self.scale(factor, factor)
        self._zoom = new_zoom
        self._auto_fit = False
        event.accept()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._auto_fit and self._map_item is not None:
            self._apply_fit()

    def _apply_fit(self):
        if self._map_item is None:
            return
        self.resetTransform()
        self.fitInView(self._map_item, Qt.KeepAspectRatio)
# -------- ROS bridge (topic list / echo) --------
class RosBridge(QObject):
    message_received = pyqtSignal(str, str, str)  # topic, type, yaml
    topics_refreshed = pyqtSignal(list)           # list of (name, type)
    error_signal = pyqtSignal(str)

    def __init__(self, node_name: str = "mu_ui_node"):
        super().__init__()
        self._node = None
        self._executor = None
        self._thread = None
        self._subs = {}  # topic -> (subscription, type_str)
        self._custom_subs = {}  # token -> subscription
        self._custom_counter = 0
        self._lock = threading.RLock()

        rclpy.init(args=None)
        self._node = rclpy.create_node(node_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=False)

    def _spin(self):
        try:
            self._executor.spin()
        except Exception as e:
            self.error_signal.emit(f"Executor stopped: {e}\n{traceback.format_exc()}")

    def shutdown(self):
        with self._lock:
            for topic in list(self._subs.keys()):
                self.unsubscribe(topic)
            for token in list(self._custom_subs.keys()):
                self.unsubscribe_typed(token)
        try:
            if self._executor and self._node:
                self._executor.remove_node(self._node)
            if self._node:
                self._node.destroy_node()
        finally:
            rclpy.shutdown()

    # Topic ops
    def refresh_topics(self):
        try:
            pairs = self._node.get_topic_names_and_types()
            flattened = []
            for name, types in pairs:
                if types:
                    flattened.append((name, types[0]))
            self.topics_refreshed.emit(sorted(flattened, key=lambda x: x[0]))
        except Exception as e:
            self.error_signal.emit(f"Failed to query topics: {e}")

    def subscribe(self, topic: str, type_str: str, depth: int = 10):
        with self._lock:
            if topic in self._subs:
                return
            try:
                msg_cls = get_message(type_str)
            except Exception as e:
                self.error_signal.emit(f"Cannot load message type '{type_str}': {e}")
                return

            qos = QoSProfile(depth=depth)

            def cb(msg):
                try:
                    yaml = message_to_yaml(msg)
                except Exception:
                    yaml = str(msg)
                self.message_received.emit(topic, type_str, yaml)

            try:
                sub = self._node.create_subscription(msg_cls, topic, cb, qos)
                self._subs[topic] = (sub, type_str)
            except Exception as e:
                self.error_signal.emit(f"Subscribe failed for {topic}: {e}")

    def unsubscribe(self, topic: str):
        with self._lock:
            tup = self._subs.pop(topic, None)
            if not tup:
                return
            sub, _ = tup
            try:
                self._node.destroy_subscription(sub)
            except Exception as e:
                self.error_signal.emit(f"Unsubscribe failed for {topic}: {e}")

    def subscribe_typed(
        self,
        topic: str,
        msg_cls,
        callback: Callable,
        depth: int = 10,
        qos: Optional[QoSProfile] = None,
    ) -> Optional[str]:
        with self._lock:
            qos_profile = qos if qos is not None else QoSProfile(depth=depth)
            try:
                sub = self._node.create_subscription(msg_cls, topic, callback, qos_profile)
            except Exception as e:
                self.error_signal.emit(f"Subscribe failed for {topic}: {e}")
                return None
            self._custom_counter += 1
            token = f"custom_{self._custom_counter}"
            self._custom_subs[token] = sub
            return token

    def unsubscribe_typed(self, token: str):
        with self._lock:
            sub = self._custom_subs.pop(token, None)
            if not sub:
                return
            try:
                self._node.destroy_subscription(sub)
            except Exception as e:
                self.error_signal.emit(f"Unsubscribe failed for subscription {token}: {e}")

    def lookup_transform(self, target_frame: str, source_frame: str):
        with self._lock:
            if not target_frame or not source_frame:
                return None
            try:
                return self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
            except Exception:
                return None
            

# -------- Main Window --------
class MuMainWindow(QMainWindow):
    def __init__(self, cfg: MuConfig, parent=None):
        super().__init__(parent)
        uic.loadUi(str(self._ui_path()), self)

        self.cfg = cfg

        original_view = self.graphicsViewMap
        self.graphicsViewMap = MapGraphicsView(original_view.parent())
        self.graphicsViewMap.setObjectName(original_view.objectName())
        self.graphicsViewMap.setMinimumSize(original_view.minimumSize())
        self.graphicsViewMap.setSizePolicy(original_view.sizePolicy())
        self.graphicsViewMap.setFrameShape(original_view.frameShape())
        self.graphicsViewMap.setFrameShadow(original_view.frameShadow())
        self.graphicsViewMap.setFocusPolicy(original_view.focusPolicy())
        self.graphicsViewMap.setStyleSheet(original_view.styleSheet())
        self.graphicsViewMap.setHorizontalScrollBarPolicy(original_view.horizontalScrollBarPolicy())
        self.graphicsViewMap.setVerticalScrollBarPolicy(original_view.verticalScrollBarPolicy())
        self.verticalLayout_cartographer.replaceWidget(original_view, self.graphicsViewMap)
        original_view.deleteLater()

        # Widgets
        self.btnBringupStart.clicked.connect(self.on_bringup_start)
        self.btnBringupStop.clicked.connect(self.on_bringup_stop)
        self.btnRefresh.clicked.connect(self.on_refresh)
        self.btnSubscribe.clicked.connect(self.on_subscribe)
        self.btnUnsubscribe.clicked.connect(self.on_unsubscribe)
        self.listTopics.itemDoubleClicked.connect(self.on_subscribe)
        self.btnCartoStart.clicked.connect(self.on_carto_start)
        self.btnCartoStop.clicked.connect(self.on_carto_stop)
        self.btnMapSubscribe.clicked.connect(self.on_map_subscribe_clicked)
        self.btnSaveMap.clicked.connect(self.on_save_map)
        self.btnResetView.clicked.connect(self.on_reset_map_view)
        self.lineMapTopic.setText(cfg.map_topic)
        self.lineMapTopic.editingFinished.connect(self.on_map_topic_edited)
        self.lineRobotFrame.setText(cfg.robot_frame)
        self.lineRobotFrame.editingFinished.connect(self.on_robot_frame_edited)
        self.btnSaveMap.setEnabled(False)

        self.map_scene = QGraphicsScene(self.graphicsViewMap)
        self.graphicsViewMap.setScene(self.map_scene)

        # Map state
        self.map_signal = MapSignal()
        self.map_signal.map_received.connect(self.on_map_message)
        self._map_pixmap_item = None
        self._map_image_data: Optional[bytearray] = None
        self._map_subscription_token: Optional[str] = None
        self._map_callback: Optional[Callable] = None
        self._latest_grid: Optional[OccupancyGrid] = None
        self._last_map_size: Optional[Tuple[int, int]] = None
        self._map_frame_id: str = "map"
        self._robot_frame: str = cfg.robot_frame
        self._robot_pose_item = None
        self._robot_heading_item = None
        self._robot_pose_timer = QTimer(self)
        self._robot_pose_timer.setInterval(200)
        self._robot_pose_timer.timeout.connect(self._update_robot_pose_visual)

        # State
        self.current_topic = None
        self.current_type = None

        # Managers
        self.bringup = BringupManager(cfg)
        self.bringup.state_changed.connect(self.on_bringup_state)
        self.bringup.error_signal.connect(self.on_error)

        self.carto = CartographerManager(cfg)
        self.carto.state_changed.connect(self.on_carto_state)
        self.carto.error_signal.connect(self.on_error)

        self.bridge = RosBridge()
        self.bridge.topics_refreshed.connect(self.on_topics_refreshed)
        self.bridge.message_received.connect(self.on_message_received)
        self.bridge.error_signal.connect(self.on_error)

        self._robot_pose_timer.start()

        self.statusbar.showMessage(
            "Ready. Use 'Bringup Start' for system launch or open the Cartographer tab to map."
        )
    @staticmethod
    def _ui_path():
        return Path(__file__).resolve().parent / "mu_ui.ui"

    # ----- Bringup handlers -----
    def on_bringup_start(self):
        self.bringup.start()

    def on_bringup_stop(self):
        self.bringup.stop()

    def on_bringup_state(self, running: bool, text: str):
        self.lblBringupStatus.setText(text)
        self.btnBringupStart.setEnabled(not running)
        self.btnBringupStop.setEnabled(running)
        self.statusbar.showMessage(text)
        # Bringup이 켜졌다면 토픽 새로고침 한번
        if running:
            self.on_refresh()

    # ----- Cartographer handlers -----
    def on_carto_start(self):
        self.statusbar.showMessage("Starting Cartographer…")
        self.carto.start()

    def on_carto_stop(self):
        self.carto.stop()

    def on_carto_state(self, running: bool, text: str):
        self.lblCartoStatus.setText(text)
        self.btnCartoStart.setEnabled(not running)
        self.btnCartoStop.setEnabled(running)
        self.statusbar.showMessage(text)
        if running:
            self._subscribe_map()

    def on_map_topic_edited(self):
        if self._map_subscription_token or self.carto.is_running():
            self._subscribe_map()

    def on_map_subscribe_clicked(self):
        self._subscribe_map()

    def _map_topic_text(self) -> str:
        text = self.lineMapTopic.text().strip()
        if not text:
            text = self.cfg.map_topic
            self.lineMapTopic.setText(text)
        return text

    def on_robot_frame_edited(self):
        text = self.lineRobotFrame.text().strip()
        if not text:
            text = self.cfg.robot_frame
        self._robot_frame = text
        self.cfg.robot_frame = text
        self.lineRobotFrame.setText(text)
        self.statusbar.showMessage(f"Robot frame: {text}")
        self._update_robot_pose_visual()

    def _subscribe_map(self):
        topic = self._map_topic_text()
        if self._map_subscription_token:
            self.bridge.unsubscribe_typed(self._map_subscription_token)
            self._map_subscription_token = None
        callback = lambda msg: self.map_signal.map_received.emit(msg)
        self._map_callback = callback
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._last_map_size = None
        token = self.bridge.subscribe_typed(topic, OccupancyGrid, callback, qos=map_qos)
        if token:
            self._map_subscription_token = token
            prefix = "Cartographer: running" if self.carto.is_running() else "Map subscription"
            self.lblCartoStatus.setText(f"{prefix} (map: {topic})")
            self.statusbar.showMessage(f"Subscribed to map topic {topic}")
        else:
            self.statusbar.showMessage(f"Failed to subscribe to {topic}")

    def _unsubscribe_map(self):
        if self._map_subscription_token:
            self.bridge.unsubscribe_typed(self._map_subscription_token)
            self._map_subscription_token = None
        self._map_callback = None

    def on_reset_map_view(self):
        self.graphicsViewMap.reset_view()
        self._update_robot_pose_visual()
        self.statusbar.showMessage("Map view reset to fit")

    def on_map_message(self, msg: OccupancyGrid):
        self._latest_grid = msg
        self.btnSaveMap.setEnabled(True)
        self.lblMapInfo.setText(self._format_map_info(msg))
        stamp_text = self._format_stamp(msg.header.stamp)
        self.lblMapUpdate.setText(f"Last update: {stamp_text}")
        frame_id = (getattr(msg.header, "frame_id", "") or "").strip()
        self._map_frame_id = frame_id if frame_id else "map"
        self._update_map_display(msg)
        self._update_robot_pose_visual()

    def _update_map_display(self, grid: OccupancyGrid):
        image = self._grid_to_qimage(grid)
        if image is None:
            return
        pixmap = QPixmap.fromImage(image)
        if self._map_pixmap_item is None:
            self._map_pixmap_item = self.map_scene.addPixmap(pixmap)
            self._map_pixmap_item.setZValue(0)
        else:
            self._map_pixmap_item.setPixmap(pixmap)
        self.map_scene.setSceneRect(QRectF(pixmap.rect()))
        size = (int(grid.info.width), int(grid.info.height))
        size_changed = self._last_map_size != size or self._map_pixmap_item is None
        self._last_map_size = size
        self.graphicsViewMap.update_map_item(self._map_pixmap_item, size_changed)

    def _grid_to_qimage(self, grid: OccupancyGrid) -> Optional[QImage]:
        width = int(grid.info.width)
        height = int(grid.info.height)
        if width <= 0 or height <= 0:
            return None
        data = grid.data
        if len(data) != width * height:
            return None
        values = bytearray(width * height)
        for idx, occ in enumerate(data):
            if occ < 0:
                val = 205
            else:
                val = max(0, min(255, int(round(255 - (occ / 100.0) * 255))))
            values[idx] = val
        flipped = bytearray(width * height)
        for y in range(height):
            src = (height - 1 - y) * width
            dst = y * width
            flipped[dst:dst + width] = values[src:src + width]
        self._map_image_data = flipped
        image = QImage(self._map_image_data, width, height, width, QImage.Format_Grayscale8)
        return image

    def _update_robot_pose_visual(self):
        if self._latest_grid is None or self._map_pixmap_item is None:
            self._set_robot_items_visible(False)
            return
        target_frame = self._map_frame_id or "map"
        transform = self.bridge.lookup_transform(target_frame, self._robot_frame)
        if transform is None:
            self._set_robot_items_visible(False)
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw_map = MuMainWindow._quaternion_to_yaw(rotation)
        coords = self._pose_to_display_coordinates(
            self._latest_grid,
            float(translation.x),
            float(translation.y),
            yaw_map,
        )
        if coords is None:
            self._set_robot_items_visible(False)
            return
        display_x, display_y, display_yaw = coords
        self._ensure_robot_items()
        self._draw_robot_marker(display_x, display_y, display_yaw)

    def _pose_to_display_coordinates(
        self,
        grid: OccupancyGrid,
        x_map: float,
        y_map: float,
        yaw_map: float,
    ) -> Optional[Tuple[float, float, float]]:
        width = float(grid.info.width)
        height = float(grid.info.height)
        resolution = float(grid.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0:
            return None
        origin = grid.info.origin
        origin_yaw = MuMainWindow._quaternion_to_yaw(origin.orientation)
        dx = x_map - float(origin.position.x)
        dy = y_map - float(origin.position.y)
        cos_o = math.cos(origin_yaw)
        sin_o = math.sin(origin_yaw)
        grid_x = (cos_o * dx + sin_o * dy) / resolution
        grid_y = (-sin_o * dx + cos_o * dy) / resolution
        if any(math.isnan(v) or math.isinf(v) for v in (grid_x, grid_y)):
            return None
        margin = 0.2
        if (
            grid_x < -width * margin
            or grid_y < -height * margin
            or grid_x > width * (1.0 + margin)
            or grid_y > height * (1.0 + margin)
        ):
            return None
        display_x = grid_x
        display_y = (height - 1.0) - grid_y
        grid_yaw = yaw_map - origin_yaw
        display_yaw = -grid_yaw
        return display_x, display_y, display_yaw

    def _ensure_robot_items(self):
        if self._robot_pose_item is None:
            pen = QPen(QColor(30, 144, 255))
            pen.setWidthF(2.0)
            pen.setCosmetic(True)
            brush = QBrush(QColor(30, 144, 255, 90))
            self._robot_pose_item = self.map_scene.addEllipse(0, 0, 0, 0, pen, brush)
            self._robot_pose_item.setZValue(10)
        if self._robot_heading_item is None:
            pen = QPen(QColor(255, 85, 0))
            pen.setWidthF(2.5)
            pen.setCosmetic(True)
            self._robot_heading_item = self.map_scene.addLine(0, 0, 0, 0, pen)
            self._robot_heading_item.setZValue(11)

    def _set_robot_items_visible(self, visible: bool):
        if self._robot_pose_item:
            self._robot_pose_item.setVisible(visible)
        if self._robot_heading_item:
            self._robot_heading_item.setVisible(visible)

    def _draw_robot_marker(self, x: float, y: float, yaw: float):
        if not self._robot_pose_item or not self._robot_heading_item:
            return
        grid = self._latest_grid
        if grid is None:
            return
        resolution = float(grid.info.resolution)
        if resolution <= 0:
            return
        radius_pixels = max(4.0, 0.25 / resolution)
        diameter = radius_pixels * 2.0
        self._robot_pose_item.setRect(x - radius_pixels, y - radius_pixels, diameter, diameter)
        heading_length = max(6.0, 0.45 / resolution)
        end_x = x + heading_length * math.cos(yaw)
        end_y = y + heading_length * math.sin(yaw)
        self._robot_heading_item.setLine(x, y, end_x, end_y)
        self._set_robot_items_visible(True)

    def _format_map_info(self, grid: OccupancyGrid) -> str:
        width = int(grid.info.width)
        height = int(grid.info.height)
        res = float(grid.info.resolution)
        frame = (getattr(grid.header, "frame_id", "") or "").strip() or "unknown"
        return f"Map: {width} x {height} cells @ {res:.3f} m (frame: {frame})"

    @staticmethod
    def _format_stamp(stamp) -> str:
        if stamp is None:
            return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            secs = float(stamp.sec) + float(stamp.nanosec) / 1e9
        except Exception:
            return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if secs <= 0:
            return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            dt = datetime.fromtimestamp(secs)
            return dt.strftime("%Y-%m-%d %H:%M:%S")
        except (OverflowError, OSError, ValueError):
            return f"{stamp.sec}.{int(stamp.nanosec):09d}"

    def on_save_map(self):
        if not self._latest_grid:
            QMessageBox.information(self, "Save Map", "No map data available yet.")
            return
        default_dir = str(self.cfg.ws_root)
        suggested = Path(default_dir) / "map.pgm"
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Map",
            str(suggested),
            "PGM Files (*.pgm)",
        )
        if not filename:
            return
        pgm_path = Path(filename)
        if pgm_path.suffix.lower() != ".pgm":
            pgm_path = pgm_path.with_suffix(".pgm")
        try:
            yaml_path = self._write_map_files(pgm_path, self._latest_grid)
        except Exception as e:
            self.on_error(f"Failed to save map: {e}")
            return
        self.statusbar.showMessage(f"Map saved to {pgm_path}")
        QMessageBox.information(
            self,
            "Save Map",
            f"Saved map to:\n{pgm_path}\n{yaml_path}",
        )

    @staticmethod
    def _write_map_files(pgm_path: Path, grid: OccupancyGrid) -> Path:
        width = int(grid.info.width)
        height = int(grid.info.height)
        if width <= 0 or height <= 0:
            raise ValueError("Map has zero dimensions")
        data = grid.data
        if len(data) != width * height:
            raise ValueError("Map data length mismatch")
        pixels = bytearray(width * height)
        for idx, occ in enumerate(data):
            if occ < 0:
                val = 205
            else:
                val = max(0, min(255, int(round(255 - (occ / 100.0) * 255))))
            pixels[idx] = val
        flipped = bytearray(width * height)
        for y in range(height):
            src = (height - 1 - y) * width
            dst = y * width
            flipped[dst:dst + width] = pixels[src:src + width]
        header = f"P5\n{width} {height}\n255\n".encode("ascii")
        pgm_path.parent.mkdir(parents=True, exist_ok=True)
        with open(pgm_path, "wb") as f:
            f.write(header)
            f.write(flipped)
        yaml_path = pgm_path.with_suffix(".yaml")
        yaw = MuMainWindow._quaternion_to_yaw(grid.info.origin.orientation)
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

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        # q is geometry_msgs.msg.Quaternion
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ----- Topic handlers -----
    def on_refresh(self):
        self.statusbar.showMessage("Refreshing topics…")
        self.bridge.refresh_topics()

    def on_topics_refreshed(self, pairs):
        self.listTopics.clear()
        for name, typ in pairs:
            item = QListWidgetItem(f"{name}    [{typ}]")
            item.setData(Qt.UserRole, (name, typ))
            self.listTopics.addItem(item)
        self.statusbar.showMessage(f"Found {len(pairs)} topics.")

    def _selected_topic_type(self):
        item = self.listTopics.currentItem()
        if not item:
            return None, None
        return item.data(Qt.UserRole)

    def on_subscribe(self):
        topic, typ = self._selected_topic_type()
        if not topic:
            QMessageBox.information(self, "Subscribe", "Select a topic first.")
            return
        depth = int(self.spinQueue.value())
        self.bridge.subscribe(topic, typ, depth=depth)
        self.current_topic = topic
        self.current_type = typ
        self.lblSelectedTopic.setText(topic)
        self.lblType.setText(typ)
        self.lblLastUpdate.setText("-")
        self.txtMessage.setPlainText("")
        self.statusbar.showMessage(f"Echoing {topic} (depth={depth})")

    def on_unsubscribe(self):
        if not self.current_topic:
            QMessageBox.information(self, "Unsubscribe", "No active subscription.")
            return
        self.bridge.unsubscribe(self.current_topic)
        self.statusbar.showMessage(f"Stopped echo on {self.current_topic}")
        self.current_topic = None
        self.current_type = None
        self.lblSelectedTopic.setText("-")
        self.lblType.setText("-")
        self.lblLastUpdate.setText("-")
        self.txtMessage.setPlainText("")

    def on_message_received(self, topic: str, type_str: str, yaml_text: str):
        if topic != self.current_topic:
            return
        self.txtMessage.setPlainText(yaml_text)
        self.lblLastUpdate.setText(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    # ----- misc -----
    def on_error(self, msg: str):
        self.statusbar.showMessage("Error")
        QMessageBox.critical(self, "Error", msg)

    def closeEvent(self, e):
        try:
            self._robot_pose_timer.stop()
        except Exception:
            pass
        try:
            self.bridge.shutdown()
        except Exception:
            pass
        try:
            self._unsubscribe_map()
        except Exception:
            pass
        try:
            self.carto.stop()
        except Exception:
            pass
        try:
            self.bringup.stop()
        except Exception:
            pass
        super().closeEvent(e)


def run_app(cfg: MuConfig):
    app = QApplication(sys.argv)
    win = MuMainWindow(cfg)
    win.show()
    sys.exit(app.exec_())


# Allow `python3 ui/mu_ui.py` direct run for debugging (with default cfg)
if __name__ == "__main__":
    # 기본값(foxy, 프로젝트 루트 추정)으로 GUI만 실행
    from mu.mu_runner.config import MuConfig as _MuConfig
    _default = _MuConfig.from_args(type("Args", (), {
        "ros_distro": None,
        "ws_root": None,
        "package": None,
        "launch_file": None,
        "no_interactive": False,
        "carto_package": None,
        "carto_launch": None,
        "map_topic": None,
    })())
    run_app(_default)

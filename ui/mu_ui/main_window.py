"""Main window for the MU PyQt5 user interface."""
from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional, Tuple

from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QColor, QBrush, QImage, QPen, QPixmap
from PyQt5.QtWidgets import QFileDialog, QListWidgetItem, QGraphicsScene, QMainWindow, QMessageBox

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from mu.mu_runner.config import MuConfig

from .managers import BringupManager, CartographerManager
from .map_utils import (
    format_map_info,
    format_stamp,
    write_map_files,
    occupancy_to_gray,
    quaternion_to_yaw,
)
from .map_view import MapGraphicsView, MapSignal
from .ros_bridge import RosBridge


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
        self.lblMapInfo.setText(format_map_info(msg))
        stamp_text = format_stamp(msg.header.stamp)
        self.lblMapUpdate.setText(f"Last update: {stamp_text}")
        frame_id = (getattr(msg.header, "frame_id", "") or "").strip()
        self._map_frame_id = frame_id if frame_id else "map"
        self._update_map_display(msg)
        self._update_robot_pose_visual()

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
            yaml_path = write_map_files(pgm_path, self._latest_grid)
        except Exception as e:
            self.on_error(f"Failed to save map: {e}")
            return
        self.statusbar.showMessage(f"Map saved to {pgm_path}")
        QMessageBox.information(
            self,
            "Save Map",
            f"Saved map to:\n{pgm_path}\n{yaml_path}",
        )

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
            values[idx] = occupancy_to_gray(occ)
        flipped = bytearray(width * height)
        for y in range(height):
            src = (height - 1 - y) * width
            dst = y * width
            flipped[dst : dst + width] = values[src : src + width]
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
        yaw_map = quaternion_to_yaw(rotation)
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
        origin_yaw = quaternion_to_yaw(origin.orientation)
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
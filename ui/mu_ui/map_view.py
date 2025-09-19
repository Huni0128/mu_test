"""Widgets used by the MU UI for map visualisation."""
from __future__ import annotations

from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtGui import QPainter
from PyQt5.QtWidgets import QGraphicsView


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
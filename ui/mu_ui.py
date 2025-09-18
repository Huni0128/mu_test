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
from datetime import datetime
from pathlib import Path
from typing import Optional, List

from PyQt5 import uic
from PyQt5.QtCore import QObject, pyqtSignal, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QListWidgetItem

# ROS 2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_yaml

# 기존 설정/환경 유틸 재사용
from mu.mu_runner.config import MuConfig
from mu.mu_runner.env import build_source_prefix


# -------- Bringup process manager --------
class BringupManager(QObject):
    state_changed = pyqtSignal(bool, str)  # running flag, status text
    error_signal = pyqtSignal(str)

    def __init__(self, cfg: MuConfig):
        super().__init__()
        self.cfg = cfg
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.RLock()

    def _build_cmd(self) -> List[str]:
        prefix = build_source_prefix(self.cfg)
        cmd = f"{prefix} && ros2 launch {self.cfg.package} {self.cfg.launch_file}"
        shell = ["bash", "-i", "-c", cmd] if self.cfg.interactive_bash else ["bash", "-c", cmd]
        return shell

    def start(self):
        with self._lock:
            if self._proc and self._proc.poll() is None:
                self.state_changed.emit(True, "Bringup already running")
                return
            try:
                shell = self._build_cmd()
                # 새 프로세스 그룹으로 실행 → 그룹 신호로 정리
                self._proc = subprocess.Popen(shell, preexec_fn=os.setsid)
                self.state_changed.emit(True, "Bringup: running")
            except Exception as e:
                self._proc = None
                self.error_signal.emit(f"Failed to start bringup: {e}")

    def stop(self, wait_seconds: float = 8.0):
        with self._lock:
            if not self._proc:
                self.state_changed.emit(False, "Bringup: stopped")
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
                self.state_changed.emit(False, "Bringup: stopped")

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None


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
        self._lock = threading.RLock()

        rclpy.init(args=None)
        self._node = rclpy.create_node(node_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception as e:
            self.error_signal.emit(f"Executor stopped: {e}\n{traceback.format_exc()}")

    def shutdown(self):
        with self._lock:
            for topic in list(self._subs.keys()):
                self.unsubscribe(topic)
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
                self.error_signal.emit(f"Subscribe failed: {e}")

    def unsubscribe(self, topic: str):
        with self._lock:
            tup = self._subs.pop(topic, None)
            if not tup:
                return
            sub, _ = tup
            try:
                self._node.destroy_subscription(sub)
            except Exception as e:
                self.error_signal.emit(f"Unsubscribe failed: {e}")


# -------- Main Window --------
class MuMainWindow(QMainWindow):
    def __init__(self, cfg: MuConfig, parent=None):
        super().__init__(parent)
        uic.loadUi(str(self._ui_path()), self)

        self.cfg = cfg

        # Widgets
        self.btnBringupStart.clicked.connect(self.on_bringup_start)
        self.btnBringupStop.clicked.connect(self.on_bringup_stop)
        self.btnRefresh.clicked.connect(self.on_refresh)
        self.btnSubscribe.clicked.connect(self.on_subscribe)
        self.btnUnsubscribe.clicked.connect(self.on_unsubscribe)
        self.listTopics.itemDoubleClicked.connect(self.on_subscribe)

        # State
        self.current_topic = None
        self.current_type = None

        # Managers
        self.bringup = BringupManager(cfg)
        self.bringup.state_changed.connect(self.on_bringup_state)
        self.bringup.error_signal.connect(self.on_error)

        self.bridge = RosBridge()
        self.bridge.topics_refreshed.connect(self.on_topics_refreshed)
        self.bridge.message_received.connect(self.on_message_received)
        self.bridge.error_signal.connect(self.on_error)

        self.statusbar.showMessage("Ready. Click 'Bringup Start' to launch, then 'Refresh Topics'.")

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
            self.bridge.shutdown()
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
        "ros_distro": None, "ws_root": None, "package": None,
        "launch_file": None, "no_interactive": False
    })())
    run_app(_default)

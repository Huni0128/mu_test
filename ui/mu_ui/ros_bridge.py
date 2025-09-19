"""Bridge between Qt and ROS 2 for the MU UI."""
from __future__ import annotations

import threading
import traceback
from typing import Callable, Optional

from PyQt5.QtCore import QObject, pyqtSignal

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.time import Time
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener


class RosBridge(QObject):
    message_received = pyqtSignal(str, str, str)  # topic, type, yaml
    topics_refreshed = pyqtSignal(list)  # list of (name, type)
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

    # Topic operations --------------------------------------------------
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
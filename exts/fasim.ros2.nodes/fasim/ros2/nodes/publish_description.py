"""Latch robot_description from robot_common_launch xacro on a ROS topic."""

from __future__ import annotations

from typing import Any


def _ns(namespace: str) -> str:
    return (namespace or "").strip().strip("/")


class RobotDescriptionPublisher:
    """Keep a transient_local robot_description publisher alive until stop()."""

    def __init__(self) -> None:
        self._context: Any = None
        self._node: Any = None
        self._publisher: Any = None
        self._topic = ""

    @property
    def topic(self) -> str:
        return self._topic

    def start(self, urdf_xml: str, namespace: str = "") -> str:
        self.stop()
        if not (urdf_xml or "").strip():
            raise RuntimeError("robot_description XML is empty")

        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        ns = _ns(namespace)
        self._context = rclpy.Context()
        rclpy.init(context=self._context, args=None)
        # Same short name; uniqueness via namespace prefix → /<ns>/fasim_robot_description
        # Relative topic "robot_description" → /<ns>/robot_description (empty ns → /robot_description).
        self._node = rclpy.create_node(
            "fasim_robot_description",
            context=self._context,
            namespace=ns or "",
        )
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._publisher = self._node.create_publisher(String, "robot_description", qos)
        self._topic = self._publisher.topic_name
        msg = String()
        msg.data = urdf_xml
        self._publisher.publish(msg)
        return self._topic

    def stop(self) -> None:
        try:
            if self._node is not None:
                if self._publisher is not None:
                    self._node.destroy_publisher(self._publisher)
                self._node.destroy_node()
        except Exception:
            pass
        self._publisher = None
        self._node = None
        try:
            if self._context is not None and self._context.ok():
                import rclpy

                rclpy.shutdown(context=self._context)
        except Exception:
            pass
        self._context = None
        self._topic = ""

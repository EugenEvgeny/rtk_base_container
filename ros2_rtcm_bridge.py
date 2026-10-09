#!/usr/bin/env python3
"""ROS 2 sidecar for the ZED-F9P base's read-only RTCM TCP and status HTTP APIs.

The receiver stays exclusively owned by gnss_base.py. This file is intentionally
importable without ROS installed, so its transport can be unit-tested offline.
"""

from __future__ import annotations

import json
import logging
import os
import socket
import threading
from urllib.request import urlopen

from gnss_base import StreamParser

LOG = logging.getLogger('rtk-base-ros2-bridge')


def read_status(url: str, timeout: float = 2.0) -> dict:
    """Read the base API and validate that it returns a JSON object."""
    with urlopen(url, timeout=timeout) as response:
        data = response.read(65537)
    if len(data) > 65536:
        raise ValueError('Status response exceeds 64 KiB')
    result = json.loads(data)
    if not isinstance(result, dict):
        raise ValueError('Status endpoint did not return a JSON object')
    return result


class RTCMStreamWorker:
    """Maintain a reconnecting, strictly read-only RTCM3 TCP connection."""

    def __init__(self, host, port, on_frame, stop_event=None,
                 reconnect_s=2.0, on_connect=None):
        self.host = host
        self.port = int(port)
        self.on_frame = on_frame
        self.stop = stop_event if stop_event is not None else threading.Event()
        self.reconnect_s = reconnect_s
        self.on_connect = on_connect if on_connect else lambda connected: None
        self.connected = False

    def run(self):
        while not self.stop.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=3.0) as sock:
                    sock.settimeout(0.5)
                    self.connected = True
                    self.on_connect(True)
                    # Reset packet framing after every reconnect. Partial frames
                    # from an old TCP connection must never contaminate new data.
                    parser = StreamParser()
                    while not self.stop.is_set():
                        try:
                            chunk = sock.recv(8192)
                        except socket.timeout:
                            continue
                        if not chunk:
                            break
                        for packet in parser.feed(chunk):
                            if packet.protocol == 'rtcm':
                                # Packet already passed RTCM CRC24Q validation.
                                self.on_frame(packet.type, packet.raw)
            except OSError as exc:
                LOG.debug('RTCM TCP connection unavailable: %s', exc)
            finally:
                if self.connected:
                    self.connected = False
                    self.on_connect(False)
            if not self.stop.is_set():
                self.stop.wait(self.reconnect_s)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rtcm_msgs.msg import Message as RTCMMessage
    from std_msgs.msg import Bool, String, UInt16

    host = os.environ.get('RTCM_HOST', 'base')
    port = int(os.environ.get('RTCM_PORT', '2102'))
    status_url = os.environ.get('STATUS_URL', 'http://base:8080/status')
    frame_id = os.environ.get('RTCM_FRAME_ID', 'rtk_base')
    status_interval = float(os.environ.get('STATUS_POLL_S', '2.0'))
    if status_interval <= 0:
        raise ValueError('STATUS_POLL_S must be positive')

    class RTKBaseBridge(Node):
        def __init__(self):
            super().__init__('rtk_base_bridge')
            # A RELIABLE writer is compatible with RELIABLE and BEST_EFFORT
            # subscribers; KEEP_LAST prevents unbounded backlog.
            rtcm_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.rtcm_pub = self.create_publisher(RTCMMessage, '/rtk_base/rtcm', rtcm_qos)
            self.type_pub = self.create_publisher(UInt16, '/rtk_base/rtcm_type', 30)
            self.status_pub = self.create_publisher(String, '/rtk_base/status', 10)
            self.ready_pub = self.create_publisher(Bool, '/rtk_base/ready', 10)
            self.stop = threading.Event()
            self.worker = RTCMStreamWorker(
                host, port, self.publish_frame, self.stop,
                on_connect=self.on_tcp_connection,
            )
            self.worker_thread = threading.Thread(
                target=self.worker.run, name='rtcm-tcp-reader', daemon=True
            )
            self.worker_thread.start()
            self.create_timer(status_interval, self.publish_status)
            self.get_logger().info(
                f'RTCM from {host}:{port}; status from {status_url}; frame_id={frame_id}'
            )

        def on_tcp_connection(self, connected):
            if connected:
                self.get_logger().info('RTCM TCP connection established')
            else:
                self.get_logger().warning('RTCM TCP disconnected; reconnecting')

        def publish_frame(self, rtcm_type: int, frame: bytes):
            msg = RTCMMessage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            # Includes D3 preamble, 10-bit length, payload AND CRC24Q. Many ROS
            # rover drivers expect a full RTCM frame, not only its payload.
            msg.message = list(frame)
            self.rtcm_pub.publish(msg)
            self.type_pub.publish(UInt16(data=rtcm_type))

        def publish_status(self):
            try:
                status = read_status(status_url)
                self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
                self.ready_pub.publish(Bool(data=status.get('base_ready') is True))
            except (OSError, ValueError, TimeoutError) as exc:
                # Keep publishing ready=false when the GNSS service is offline.
                # A monitoring client must never interpret silence as readiness.
                self.ready_pub.publish(Bool(data=False))
                self.status_pub.publish(String(data=json.dumps({
                    'base_ready': False, 'error': str(exc)
                })))
                self.get_logger().warning(f'Base status unavailable: {exc}')

        def shutdown(self):
            self.stop.set()
            self.worker_thread.join(timeout=5)

    rclpy.init()
    node = RTKBaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

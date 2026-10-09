"""Transport-level ROS 2 bridge tests; no Docker, ROS or receiver required."""

import json
import socket
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gnss_base import crc24q
from ros2_rtcm_bridge import RTCMStreamWorker, read_status


def rtcm_frame(message_type: int) -> bytes:
    payload = bytes([message_type >> 4, (message_type & 15) << 4, 0x33, 0x44])
    body = b'\xd3\x00' + bytes([len(payload)]) + payload
    return body + crc24q(body).to_bytes(3, 'big')


class TestRTCMStreamWorker(unittest.TestCase):
    def test_recovers_fragmented_packets_and_ignores_bad_crc(self):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        expected1, expected2 = rtcm_frame(1005), rtcm_frame(1074)
        received = []
        got_two = threading.Event()
        stop = threading.Event()
        connection_events = []

        def on_frame(kind, packet):
            received.append((kind, packet))
            if len(received) == 2:
                got_two.set()

        worker = RTCMStreamWorker('127.0.0.1', port, on_frame, stop,
                                  reconnect_s=0.01,
                                  on_connect=connection_events.append)
        t = threading.Thread(target=worker.run)
        t.start()
        try:
            conn, _ = listener.accept()
            with conn:
                bad = expected1[:-1] + bytes([expected1[-1] ^ 0xff])
                conn.sendall(expected1[:3])
                conn.sendall(expected1[3:] + bad + expected2[:2])
                conn.sendall(expected2[2:])
                self.assertTrue(got_two.wait(2), 'RTCM frames not delivered')
            self.assertEqual(received, [(1005, expected1), (1074, expected2)])
            self.assertTrue(connection_events[0])
        finally:
            stop.set()
            t.join(2)
            listener.close()
        self.assertFalse(t.is_alive())

    def test_reconnect_discards_unfinished_frame(self):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(2)
        listener.settimeout(2)
        port = listener.getsockname()[1]
        frame = rtcm_frame(1230)
        received = []
        event = threading.Event()
        stop = threading.Event()

        def on_frame(kind, packet):
            received.append((kind, packet))
            event.set()

        worker = RTCMStreamWorker('127.0.0.1', port, on_frame,
                                  stop_event=stop, reconnect_s=0.01)
        t = threading.Thread(target=worker.run)
        t.start()
        try:
            conn1, _ = listener.accept()
            with conn1:
                conn1.sendall(frame[:5])
            conn2, _ = listener.accept()
            with conn2:
                conn2.sendall(frame)
                self.assertTrue(event.wait(2), 'No frame after reconnect')
            self.assertEqual(received, [(1230, frame)])
        finally:
            stop.set()
            t.join(2)
            listener.close()
        self.assertFalse(t.is_alive())

    def test_handles_unavailable_tcp_endpoint(self):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        port = listener.getsockname()[1]
        listener.close()
        stop = threading.Event()
        worker = RTCMStreamWorker('127.0.0.1', port, lambda *_: None,
                                  stop_event=stop, reconnect_s=0.02)
        t = threading.Thread(target=worker.run)
        t.start()
        time.sleep(0.10)
        stop.set()
        t.join(2)
        self.assertFalse(t.is_alive())
        self.assertFalse(worker.connected)


class TestHTTPStatus(unittest.TestCase):
    def test_status_json_and_invalid_payload(self):
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                body = (json.dumps({'base_ready': True, 'rtcm_messages': {'1005': 1}})
                        if self.path == '/status' else '[]').encode()
                self.send_response(200)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *_):
                pass

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = 'http://127.0.0.1:%d' % server.server_address[1]
        try:
            self.assertTrue(read_status(url + '/status')['base_ready'])
            with self.assertRaises(ValueError):
                read_status(url + '/invalid')
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)


if __name__ == '__main__':
    unittest.main()

class TestROSPublication(unittest.TestCase):
    def test_ros2_message_fields_topics_and_status(self):
        """Exercise ROS publication with minimal rclpy/msg stand-ins.

        This is not a DDS test; it verifies that the bridge builds the correct
        standard ROS message objects and publishes the entire RTCM frame.
        """
        import os
        import types
        from unittest.mock import patch
        import ros2_rtcm_bridge

        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(2)
        port = listener.getsockname()[1]
        frame = rtcm_frame(1005)
        published = {}

        class FakeMessage:
            def __init__(self, data=None):
                self.data = data
                self.header = types.SimpleNamespace(stamp=None, frame_id='')
                self.message = []

        class Publisher:
            def __init__(self, topic):
                self.topic = topic
                self.messages = published.setdefault(topic, [])

            def publish(self, msg):
                self.messages.append(msg)

        class FakeNode:
            def __init__(self, name):
                self.name = name

            def create_publisher(self, message_type, topic, qos):
                return Publisher(topic)

            def create_timer(self, period, callback):
                self.status_callback = callback

            def get_logger(self):
                return types.SimpleNamespace(info=lambda *_: None, warning=lambda *_: None)

            def get_clock(self):
                return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 'time'))

            def destroy_node(self):
                pass

        qos = types.ModuleType('rclpy.qos')
        qos.QoSProfile = lambda **kwargs: kwargs
        qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
        qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
        qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
        node_mod = types.ModuleType('rclpy.node')
        node_mod.Node = FakeNode
        rtcm_mod = types.ModuleType('rtcm_msgs.msg')
        rtcm_mod.Message = FakeMessage
        std_mod = types.ModuleType('std_msgs.msg')
        std_mod.Bool = std_mod.String = std_mod.UInt16 = FakeMessage
        rclpy = types.ModuleType('rclpy')
        rclpy.init = lambda: None
        rclpy.ok = lambda: True
        rclpy.shutdown = lambda: None

        def fake_spin(node):
            # Make the status timer produce the values already available in
            # the GNSS service, then send a valid correction to the TCP reader.
            node.publish_status()
            with listener.accept()[0] as conn:
                conn.sendall(frame[:2])
                conn.sendall(frame[2:])
                deadline = time.monotonic() + 2
                while not published.get('/rtk_base/rtcm') and time.monotonic() < deadline:
                    time.sleep(0.01)
            self.assertTrue(published.get('/rtk_base/rtcm'))

        rclpy.spin = fake_spin
        environment = {'RTCM_HOST': '127.0.0.1', 'RTCM_PORT': str(port)}
        modules = {
            'rclpy': rclpy, 'rclpy.node': node_mod, 'rclpy.qos': qos,
            'rtcm_msgs': types.ModuleType('rtcm_msgs'),
            'rtcm_msgs.msg': rtcm_mod,
            'std_msgs': types.ModuleType('std_msgs'),
            'std_msgs.msg': std_mod,
        }
        try:
            with patch.dict(sys.modules, modules), patch.dict(os.environ, environment), \
                    patch.object(ros2_rtcm_bridge, 'read_status', return_value={
                        'base_ready': True, 'rtcm_bytes': 120
                    }):
                ros2_rtcm_bridge.main()
        finally:
            listener.close()

        self.assertEqual(published['/rtk_base/rtcm'][0].message, list(frame))
        self.assertEqual(published['/rtk_base/rtcm'][0].header.frame_id, 'rtk_base')
        self.assertEqual(published['/rtk_base/rtcm'][0].header.stamp, 'time')
        self.assertEqual(published['/rtk_base/rtcm_type'][0].data, 1005)
        self.assertTrue(published['/rtk_base/ready'][0].data)
        self.assertIn('rtcm_bytes', published['/rtk_base/status'][0].data)

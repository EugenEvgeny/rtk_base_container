"""Bridge tests with sockets and a pseudo-terminal; no Mac, GNSS or Docker needed."""
from __future__ import annotations

import os
import pty
import select
import socket
import sys
import threading
import time
import tty
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gnss_base import (GNSSDevice, IncomingTCPSerial, KEYS, StreamParser,
                       configure_device, open_port, ubx_frame)
from scripts.mac_serial_bridge import forward_serial
from test_base import FakePort


def unused_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def connect_retry(port, seconds=3):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=0.5)
        except ConnectionRefusedError:
            time.sleep(0.01)
    raise AssertionError("TCP serial bridge did not start listening")


class FdSerial:
    """Minimal stand-in for pyserial backed by a POSIX pseudo-terminal."""
    def __init__(self, fd):
        self.fd = fd

    def fileno(self):
        return self.fd

    @property
    def in_waiting(self):
        return 4096

    def read(self, n):
        return os.read(self.fd, n)

    def write(self, data):
        return os.write(self.fd, data)


class MacBridgeTests(unittest.TestCase):
    def test_incoming_tcp_serial_handles_real_ubx_request(self):
        port = unused_port()
        got = {}

        def probe():
            try:
                with open_port(f"listen://127.0.0.1:{port}", 38400) as link:
                    got["version"] = GNSSDevice(link, timeout=2).mon_ver()
            except Exception as exc:
                got["error"] = exc

        t = threading.Thread(target=probe)
        t.start()
        with connect_retry(port) as client:
            request = client.recv(4096)
            self.assertEqual(request, ubx_frame(0x0A, 0x04))
            response = (b"EXT CORE 1.00".ljust(30, b"\x00")
                        + b"00190000".ljust(10, b"\x00")
                        + b"MOD=ZED-F9P".ljust(30, b"\x00")
                        + b"PROTVER=27.50".ljust(30, b"\x00"))
            reply = ubx_frame(0x0A, 0x04, response)
            client.sendall(reply[:7])
            client.sendall(reply[7:])
        t.join(3)
        self.assertFalse(t.is_alive())
        self.assertNotIn("error", got)
        self.assertIn("MOD=ZED-F9P", got["version"]["extensions"])

    def test_bidirectional_forwarding_over_pty(self):
        master, slave = pty.openpty()
        try:
            tty.setraw(slave)
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            right = socket.create_connection(listener.getsockname(), timeout=2)
            left, _ = listener.accept()
            listener.close()
            result = {}

            def bridge():
                try:
                    result["sizes"] = forward_serial(FdSerial(slave), left)
                except Exception as exc:
                    result["error"] = exc
                finally:
                    left.close()

            t = threading.Thread(target=bridge)
            t.start()
            try:
                # GNSS serial -> TCP -> container
                os.write(master, b"UBX-RTCM\x00\xd3")
                right.settimeout(2)
                self.assertEqual(right.recv(256), b"UBX-RTCM\x00\xd3")
                # container -> TCP -> GNSS serial
                right.sendall(b"\xb5\x62\x0a\x04")
                r, _, _ = select.select([master], [], [], 2)
                self.assertTrue(r, "no bytes reached the physical serial side")
                self.assertEqual(os.read(master, 100), b"\xb5\x62\x0a\x04")
            finally:
                right.close()
            t.join(3)
            self.assertFalse(t.is_alive())
            self.assertNotIn("error", result)
            self.assertEqual(result["sizes"], (10, 4))
        finally:
            os.close(master)
            os.close(slave)

    def test_disconnect_is_error_not_endless_timeout(self):
        port = unused_port()
        outcome = {}

        def wait_read():
            try:
                with IncomingTCPSerial(f"listen://127.0.0.1:{port}", 2) as link:
                    link.read(1)
            except Exception as exc:
                outcome["error"] = exc

        t = threading.Thread(target=wait_read)
        t.start()
        with connect_retry(port):
            pass
        t.join(3)
        self.assertFalse(t.is_alive())
        self.assertIsInstance(outcome.get("error"), OSError)
        self.assertIn("disconnected", str(outcome["error"]))

    def test_bad_listener_url_is_rejected(self):
        for url in ("listen://localhost:1234", "listen://0.0.0.0", "listen://0.0.0.0:5/path"):
            with self.subTest(url=url), self.assertRaises(ValueError):
                IncomingTCPSerial(url, 0.1)

    def test_survey_configuration_through_full_tcp_serial_bridge(self):
        """End to end: Docker-side UBX -> TCP -> host serial -> mock ZED-F9P."""
        master, slave = pty.openpty()
        tty.setraw(slave)
        port = unused_port()
        stop = threading.Event()
        fake_receiver = FakePort()
        received = {}

        def mock_f9p():
            parser = StreamParser()
            try:
                while not stop.is_set():
                    if not select.select([master], [], [], 0.1)[0]:
                        continue
                    for message in parser.feed(os.read(master, 4096)):
                        if message.protocol == "ubx":
                            fake_receiver.write(message.raw)
                            answer = bytes(fake_receiver.rx)
                            fake_receiver.rx.clear()
                            os.write(master, answer)
            except OSError:
                pass

        def docker_command():
            try:
                with IncomingTCPSerial(f"listen://127.0.0.1:{port}", 2) as link:
                    dev = GNSSDevice(link, timeout=2)
                    configure_device(dev, SimpleNamespace(
                        action="survey", duration=600, accuracy_m=5,
                        uart2=False, uart2_baud=115200))
                    received.update(dev.valget(["CFG-TMODE-MODE", "CFG-TMODE-SVIN_MIN_DUR"]))
            except Exception as exc:
                received["error"] = exc

        bridge_error = {}
        def mac_host(client):
            try:
                forward_serial(FdSerial(slave), client)
            except Exception as exc:
                bridge_error["error"] = exc
            finally:
                client.close()

        t_device = threading.Thread(target=mock_f9p)
        t_docker = threading.Thread(target=docker_command)
        t_device.start()
        t_docker.start()
        client = connect_retry(port)
        t_bridge = threading.Thread(target=mac_host, args=(client,))
        t_bridge.start()
        try:
            t_docker.join(8)
            self.assertFalse(t_docker.is_alive(), "Docker command timed out through bridge")
            self.assertNotIn("error", received)
            self.assertEqual(received["CFG-TMODE-MODE"], 1)
            self.assertEqual(received["CFG-TMODE-SVIN_MIN_DUR"], 600)
            self.assertEqual(fake_receiver.settings[KEYS["CFG-TMODE-MODE"]], 1)
        finally:
            stop.set()
            client.close()
            t_bridge.join(3)
            t_device.join(3)
            os.close(master)
            os.close(slave)

    def test_mac_compose_does_not_require_linux_usb_mapping(self):
        path = Path(__file__).resolve().parents[1] / "compose.mac.yaml"
        conf = path.read_text()
        self.assertIn("listen://0.0.0.0:", conf)
        self.assertIn('"127.0.0.1:${GNSS_BRIDGE_PORT', conf)
        self.assertNotIn("    devices:", conf)
        self.assertNotIn("    group_add:", conf)


if __name__ == "__main__":
    unittest.main()

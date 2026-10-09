#!/usr/bin/env python3
"""Stationary ZED-F9P GNSS base configuration and RTCM3 streamer.

Targets a SparkFun GPS-RTK-SMA breakout over USB. No gpsd, ubxtool, or
RTKLIB requirement. Needs pyserial on the physical machine/container.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP
import json
import logging
import os
import queue
import select
import signal
import socket
import ssl
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

try:
    import serial
except ImportError:  # allow offline protocol unit tests without hardware/deps
    serial = None

LOG = logging.getLogger("f9p-base")

# u-blox F9 configuration database IDs; see ZED-F9P interface description.
# Keys with signed values are explicitly listed in SIGNED_KEYS below.
KEYS = {
    "CFG-TMODE-MODE": 0x20030001,
    "CFG-TMODE-POS_TYPE": 0x20030002,
    "CFG-TMODE-ECEF_X": 0x40030003,
    "CFG-TMODE-ECEF_Y": 0x40030004,
    "CFG-TMODE-ECEF_Z": 0x40030005,
    "CFG-TMODE-ECEF_X_HP": 0x20030006,
    "CFG-TMODE-ECEF_Y_HP": 0x20030007,
    "CFG-TMODE-ECEF_Z_HP": 0x20030008,
    "CFG-TMODE-FIXED_POS_ACC": 0x4003000F,
    "CFG-TMODE-SVIN_MIN_DUR": 0x40030010,
    "CFG-TMODE-SVIN_ACC_LIMIT": 0x40030011,
    "CFG-RATE-MEAS": 0x30210001,
    "CFG-RATE-NAV": 0x30210002,
    "CFG-USBOUTPROT-UBX": 0x10780001,
    "CFG-USBOUTPROT-NMEA": 0x10780002,
    "CFG-USBOUTPROT-RTCM3X": 0x10780004,
    "CFG-MSGOUT-UBX_NAV_SVIN_USB": 0x2091008B,
    "CFG-UART2-BAUDRATE": 0x40530001,
    "CFG-UART2OUTPROT-RTCM3X": 0x10760004,
    "CFG-UART2OUTPROT-NMEA": 0x10760002,
    "CFG-MSGOUT-RTCM_3X_TYPE1005_USB": 0x209102C0,
    "CFG-MSGOUT-RTCM_3X_TYPE1006_USB": 0x209102C5,
    "CFG-MSGOUT-RTCM_3X_TYPE1077_USB": 0x209102CF,
    "CFG-MSGOUT-RTCM_3X_TYPE1087_USB": 0x209102D4,
    "CFG-MSGOUT-RTCM_3X_TYPE1097_USB": 0x2091031B,
    "CFG-MSGOUT-RTCM_3X_TYPE1127_USB": 0x209102D9,
    "CFG-MSGOUT-RTCM_3X_TYPE1074_USB": 0x20910361,
    "CFG-MSGOUT-RTCM_3X_TYPE1084_USB": 0x20910366,
    "CFG-MSGOUT-RTCM_3X_TYPE1094_USB": 0x2091036B,
    "CFG-MSGOUT-RTCM_3X_TYPE1124_USB": 0x20910370,
    "CFG-MSGOUT-RTCM_3X_TYPE1230_USB": 0x20910306,
    "CFG-MSGOUT-RTCM_3X_TYPE1005_UART2": 0x209102BF,
    "CFG-MSGOUT-RTCM_3X_TYPE1006_UART2": 0x209102C4,
    "CFG-MSGOUT-RTCM_3X_TYPE1077_UART2": 0x209102CE,
    "CFG-MSGOUT-RTCM_3X_TYPE1087_UART2": 0x209102D3,
    "CFG-MSGOUT-RTCM_3X_TYPE1097_UART2": 0x2091031A,
    "CFG-MSGOUT-RTCM_3X_TYPE1127_UART2": 0x209102D8,
    "CFG-MSGOUT-RTCM_3X_TYPE1074_UART2": 0x20910360,
    "CFG-MSGOUT-RTCM_3X_TYPE1084_UART2": 0x20910365,
    "CFG-MSGOUT-RTCM_3X_TYPE1094_UART2": 0x2091036A,
    "CFG-MSGOUT-RTCM_3X_TYPE1124_UART2": 0x2091036F,
    "CFG-MSGOUT-RTCM_3X_TYPE1230_UART2": 0x20910305,
}
SIGNED_KEYS = {
    KEYS[f"CFG-TMODE-ECEF_{axis}"] for axis in "XYZ"
} | {KEYS[f"CFG-TMODE-ECEF_{axis}_HP"] for axis in "XYZ"}
RTCM_RATES = {1005: 5, 1074: 1, 1084: 1, 1094: 1, 1124: 1, 1230: 5}
# Disable alternatives if the receiver was previously configured for MSM7/1006.
RTCM_DISABLE = (1006, 1077, 1087, 1097, 1127)


def pack_value(key: int, value: int) -> bytes:
    width_code = key >> 28
    if width_code not in (1, 2, 3, 4):
        raise ValueError(f"Unsupported configuration key 0x{key:08X}")
    width = {1: 1, 2: 1, 3: 2, 4: 4}[width_code]
    return int(value).to_bytes(width, "little", signed=(key in SIGNED_KEYS))


def unpack_value(key: int, data: bytes) -> tuple[int, int]:
    width_code = key >> 28
    if width_code not in (1, 2, 3, 4):
        raise ValueError(f"Unsupported configuration key 0x{key:08X}")
    width = {1: 1, 2: 1, 3: 2, 4: 4}[width_code]
    if len(data) < width:
        raise ValueError("Truncated CFG-VALGET value")
    return int.from_bytes(data[:width], "little", signed=key in SIGNED_KEYS), width


def ubx_checksum(data: bytes) -> bytes:
    a, b = 0, 0
    for byte in data:
        a = (a + byte) & 255
        b = (b + a) & 255
    return bytes((a, b))


def ubx_frame(cls: int, mid: int, payload: bytes = b"") -> bytes:
    body = struct.pack("<BBH", cls, mid, len(payload)) + payload
    return b"\xb5\x62" + body + ubx_checksum(body)


def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


@dataclass
class Packet:
    protocol: str
    type: int
    payload: bytes
    raw: bytes
    msg_class: int = -1


class StreamParser:
    """Demultiplex binary RTCM3 and UBX, discarding NMEA/noise and bad CRCs."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[Packet]:
        self.buffer.extend(data)
        results = []
        b = self.buffer
        while b:
            if b[0] == 0xD3:
                if len(b) < 3:
                    break
                if b[1] & 0xFC:
                    del b[0]
                    continue
                n = ((b[1] & 3) << 8) | b[2]
                if len(b) < n + 6:
                    break
                frame = bytes(b[:n + 6])
                if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big") and n >= 2:
                    typ = (frame[3] << 4) | (frame[4] >> 4)
                    results.append(Packet("rtcm", typ, frame[3:-3], frame))
                    del b[:n + 6]
                else:
                    del b[0]
            elif b[0] == 0xB5:
                if len(b) < 2:
                    break
                if b[1] != 0x62:
                    del b[0]
                    continue
                if len(b) < 6:
                    break
                n = int.from_bytes(b[4:6], "little")
                if n > 4096:
                    del b[0]
                    continue
                if len(b) < n + 8:
                    break
                frame = bytes(b[:n + 8])
                if ubx_checksum(frame[2:-2]) == frame[-2:]:
                    results.append(Packet("ubx", frame[3], frame[6:-2], frame, frame[2]))
                    del b[:n + 8]
                else:
                    del b[0]
            else:
                del b[0]
        return results


class GNSSDevice:
    def __init__(self, port, timeout: float = 3.0):
        self.port = port
        self.timeout = timeout
        self.parser = StreamParser()

    def _wait(self, predicate, timeout=None):
        end = time.monotonic() + (timeout or self.timeout)
        while time.monotonic() < end:
            data = self.port.read(4096)
            if not data:
                continue
            for packet in self.parser.feed(data):
                if predicate(packet):
                    return packet
        raise TimeoutError("No matching UBX response; verify serial permissions, cable and receiver")

    def _request(self, cls: int, mid: int, payload=b"", predicate=None):
        # Configuration/monitor commands must never share serial ownership
        # with 'serve'. Drop buffered asynchronous output before each poll.
        self.port.reset_input_buffer()
        self.parser = StreamParser()
        self.port.write(ubx_frame(cls, mid, payload))
        if predicate is None:
            predicate = lambda p: p.protocol == "ubx" and p.msg_class == cls and p.type == mid
        return self._wait(predicate)

    def mon_ver(self) -> dict:
        pkt = self._request(0x0A, 0x04)
        d = pkt.payload
        if len(d) < 40:
            raise ValueError("Short MON-VER response")
        dec = lambda s: s.split(b"\x00", 1)[0].decode("ascii", "replace").strip()
        result = {"software": dec(d[:30]), "hardware": dec(d[30:40])}
        result["extensions"] = [dec(d[i:i + 30]) for i in range(40, len(d), 30)]
        return result

    def valset(self, configs: dict[str, int], layers: int = 7):
        if not configs:
            return
        payload = bytearray((0, layers, 0, 0))
        for name, value in configs.items():
            key = KEYS[name]
            payload.extend(struct.pack("<I", key))
            payload.extend(pack_value(key, value))
        if len(payload) > 1024:
            raise ValueError("Too many CFG-VALSET keys in one packet")
        pkt = self._request(
            0x06, 0x8A, bytes(payload),
            predicate=lambda p: p.protocol == "ubx" and p.msg_class == 0x05
            and p.type in (0x00, 0x01) and p.payload == b"\x06\x8a",
        )
        if pkt.type == 0x00:
            raise ValueError(f"Receiver rejected CFG-VALSET (layers={layers}); keys: {list(configs)}")

    def valget(self, names: list[str]) -> dict[str, int]:
        request = b"\x00\x00\x00\x00" + b"".join(struct.pack("<I", KEYS[n]) for n in names)
        pkt = self._request(0x06, 0x8B, request)
        if len(pkt.payload) < 4:
            raise ValueError("Short CFG-VALGET response")
        cursor = 4
        reverse = {v: k for k, v in KEYS.items()}
        result = {}
        while cursor + 4 <= len(pkt.payload):
            key = struct.unpack_from("<I", pkt.payload, cursor)[0]
            cursor += 4
            if key not in reverse:
                raise ValueError(f"Unknown key in CFG-VALGET: 0x{key:08x}")
            value, size = unpack_value(key, pkt.payload[cursor:])
            cursor += size
            result[reverse[key]] = value
        if cursor != len(pkt.payload):
            raise ValueError("Malformed CFG-VALGET response")
        return result

    def survey_status(self):
        pkt = self._request(0x01, 0x3B)
        return decode_svin(pkt.payload)


def decode_svin(payload: bytes) -> dict:
    if len(payload) < 40:
        raise ValueError("Short UBX-NAV-SVIN response")
    xyz = []
    for j, off in enumerate((12, 16, 20)):
        cm = struct.unpack_from("<i", payload, off)[0]
        hp = struct.unpack_from("<b", payload, 24 + j)[0]
        xyz.append(round(cm * 0.01 + hp * 0.0001, 4))
    return {
        "duration_s": struct.unpack_from("<I", payload, 8)[0],
        "accuracy_m_estimate": round(struct.unpack_from("<I", payload, 28)[0] * 0.0001, 4),
        "observation_count": struct.unpack_from("<I", payload, 32)[0],
        "valid": bool(payload[36]),
        "active": bool(payload[37]),
        "ecef_m": xyz,
    }


def split_ecef(meters: float) -> tuple[int, int]:
    value = Decimal(str(meters))
    cm = int((value * 100).to_integral_value(rounding=ROUND_HALF_UP))
    hp = int(((value - Decimal(cm) / 100) * 10000).to_integral_value(rounding=ROUND_HALF_UP))
    if not -(2**31) <= cm < 2**31 or not -99 <= hp <= 99:
        raise ValueError("ECEF value outside ZED-F9P supported range")
    return cm, hp


def accuracy_units(meters: float) -> int:
    if not 0 < meters < 100_000:
        raise ValueError("Accuracy must be positive and less than 100 km")
    result = int((Decimal(str(meters)) * 10000).to_integral_value(rounding=ROUND_HALF_UP))
    if not 0 < result <= 0xFFFFFFFF:
        raise ValueError("Accuracy cannot be represented in 0.1 mm units")
    return result


def base_config() -> dict[str, int]:
    config = {
        "CFG-RATE-MEAS": 1000,
        "CFG-RATE-NAV": 1,
        "CFG-USBOUTPROT-UBX": 1,
        "CFG-USBOUTPROT-NMEA": 0,
        "CFG-USBOUTPROT-RTCM3X": 1,
        # Survey-in status is polled explicitly rather than sent periodically.
        "CFG-MSGOUT-UBX_NAV_SVIN_USB": 0,
    }
    for typ, rate in RTCM_RATES.items():
        config[f"CFG-MSGOUT-RTCM_3X_TYPE{typ}_USB"] = rate
    for typ in RTCM_DISABLE:
        config[f"CFG-MSGOUT-RTCM_3X_TYPE{typ}_USB"] = 0
    return config


def configure_device(dev: GNSSDevice, args):
    ver = dev.mon_ver()
    LOG.info("Receiver: %s; extensions: %s", ver["software"], ver["extensions"])
    if not any("ZED-F9P" in item for item in ver["extensions"]):
        LOG.warning("MON-VER does not explicitly identify ZED-F9P; check this is the intended receiver")
    # Protect from other modes while changing reference position settings.
    write_persistent(dev, {"CFG-TMODE-MODE": 0})
    config = base_config()
    if args.uart2:
        config["CFG-UART2-BAUDRATE"] = args.uart2_baud
        config["CFG-UART2OUTPROT-RTCM3X"] = 1
        config["CFG-UART2OUTPROT-NMEA"] = 0
        for typ, rate in RTCM_RATES.items():
            config[f"CFG-MSGOUT-RTCM_3X_TYPE{typ}_UART2"] = rate
        for typ in RTCM_DISABLE:
            config[f"CFG-MSGOUT-RTCM_3X_TYPE{typ}_UART2"] = 0

    if args.action == "survey":
        config["CFG-TMODE-SVIN_MIN_DUR"] = args.duration
        config["CFG-TMODE-SVIN_ACC_LIMIT"] = accuracy_units(args.accuracy_m)
        mode = 1
    else:
        config["CFG-TMODE-POS_TYPE"] = 0  # ECEF rather than LLH
        config["CFG-TMODE-FIXED_POS_ACC"] = accuracy_units(args.accuracy_m)
        for axis, meters in zip("XYZ", (args.ecef_x, args.ecef_y, args.ecef_z)):
            cm, hp = split_ecef(meters)
            config[f"CFG-TMODE-ECEF_{axis}"] = cm
            config[f"CFG-TMODE-ECEF_{axis}_HP"] = hp
        mode = 2

    # Smaller chunks are gentler on USB receivers with high existing output rates.
    entries = list(config.items())
    for i in range(0, len(entries), 12):
        batch = dict(entries[i:i + 12])
        write_persistent(dev, batch)
        confirm = dev.valget(list(batch))
        mismatches = {n: (v, confirm.get(n)) for n, v in batch.items() if confirm.get(n) != v}
        if mismatches:
            raise RuntimeError(f"Configuration readback mismatch: {mismatches}")
    write_persistent(dev, {"CFG-TMODE-MODE": mode})
    observed = dev.valget(["CFG-TMODE-MODE"])
    if observed.get("CFG-TMODE-MODE") != mode:
        raise RuntimeError("Failed to enable requested base-station mode")
    LOG.info("Configured ZED-F9P %s base, USB RTCM MSM4 output at 1 Hz", "survey-in" if mode == 1 else "fixed")
    if mode == 1:
        LOG.info("Survey will restart when the GNSS module reboots. This is not an absolute survey.")
    else:
        LOG.info("Fixed base coordinate accuracy is only as good as the surveyed ARP coordinates supplied.")


def write_persistent(dev, entries: dict[str, int]):
    # RAM+BBR+Flash, fall back if an actual memory layer is unavailable.
    # Always warn if permanent persistence is not available.
    for layers in (7, 3, 1):
        try:
            dev.valset(entries, layers)
            if layers != 7:
                LOG.warning("Saved to memory layers mask %d (7=RAM+BBR+Flash); Flash may be unsupported", layers)
            return
        except ValueError as exc:
            LOG.warning("Configuration with layers=%d rejected: %s", layers, exc)
    raise RuntimeError("Receiver rejected configuration on all memory layers")


class RTCMTCPServer:
    def __init__(self, port: int):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("0.0.0.0", port))
        self.listener.listen(16)
        self.listener.settimeout(0.5)
        self.clients: list[socket.socket] = []
        self.lock = threading.Lock()

    def accept_loop(self, stop):
        while not stop.is_set():
            try:
                client, addr = self.listener.accept()
                client.settimeout(0.05)
                with self.lock:
                    self.clients.append(client)
                LOG.info("RTCM TCP client connected: %s", addr[0])
            except socket.timeout:
                continue
            except OSError:
                break

    def broadcast(self, frame: bytes):
        with self.lock:
            for client in self.clients[:]:
                try:
                    client.sendall(frame)
                except (OSError, socket.timeout):
                    self.clients.remove(client)
                    client.close()

    def count(self):
        with self.lock:
            return len(self.clients)

    def close(self):
        self.listener.close()
        with self.lock:
            for client in self.clients:
                client.close()
            self.clients.clear()


class NtripPublisher:
    """Push to a pre-existing NTRIP v1 caster using the SOURCE protocol."""

    def __init__(self, host, port, mount, password, tls=False):
        self.host, self.port = host, port
        self.mount, self.password, self.tls = mount, password, tls
        self.queue = queue.Queue(maxsize=256)
        self.published = False
        self.drop_count = 0

    def send_later(self, packet: bytes):
        try:
            self.queue.put_nowait(packet)
        except queue.Full:
            try:
                self.queue.get_nowait()
                self.drop_count += 1
            except queue.Empty:
                pass
            self.queue.put_nowait(packet)

    def _connect(self):
        sock = socket.create_connection((self.host, self.port), timeout=5)
        try:
            if self.tls:
                sock = ssl.create_default_context().wrap_socket(sock, server_hostname=self.host)
            sock.settimeout(5)
            auth = f"SOURCE {self.password} /{self.mount}\r\nSource-Agent: NTRIP F9P-RTK-Base/1.0\r\n\r\n"
            sock.sendall(auth.encode("ascii"))
            line = b""
            while not line.endswith(b"\n") and len(line) < 1024:
                part = sock.recv(1)
                if not part:
                    break
                line += part
            if not (line.startswith(b"ICY 200 ") or line.startswith(b"HTTP/1.0 200 ") or line.startswith(b"HTTP/1.1 200 ")):
                raise ConnectionError(f"Caster rejected source: {line[:120]!r}")
            # RTCM frames are written immediately once caster accepts.
            sock.settimeout(3)
            return sock
        except BaseException:
            sock.close()
            raise

    def loop(self, stop):
        while not stop.is_set():
            try:
                with self._connect() as sock:
                    self.published = True
                    LOG.info("NTRIP source connected to %s:%d/%s", self.host, self.port, self.mount)
                    # Drop stale corrections buffered before reconnection.
                    while True:
                        try:
                            self.queue.get_nowait()
                        except queue.Empty:
                            break
                    while not stop.is_set():
                        try:
                            frame = self.queue.get(timeout=0.5)
                        except queue.Empty:
                            continue
                        sock.sendall(frame)
            except (OSError, ValueError, ConnectionError) as exc:
                LOG.warning("NTRIP upstream unavailable: %s (reconnecting)", exc)
            finally:
                self.published = False
            stop.wait(5)


class RuntimeState:
    def __init__(self, tcp, publisher=None):
        self.tcp = tcp
        self.publisher = publisher
        self.start = time.time()
        self.last_rtcm = 0.0
        self.last_1005 = 0.0
        self.last_survey = None
        self.counts = Counter()
        self.bytes = 0
        self.lock = threading.Lock()

    def process(self, packet: Packet):
        with self.lock:
            if packet.protocol == "rtcm":
                self.counts[packet.type] += 1
                self.bytes += len(packet.raw)
                self.last_rtcm = time.time()
                if packet.type == 1005:
                    self.last_1005 = self.last_rtcm
                self.tcp.broadcast(packet.raw)
                if self.publisher:
                    self.publisher.send_later(packet.raw)
            elif packet.protocol == "ubx" and packet.msg_class == 1 and packet.type == 0x3B:
                try:
                    self.last_survey = decode_svin(packet.payload)
                except ValueError:
                    pass

    def status(self):
        now = time.time()
        with self.lock:
            return {
                "uptime_s": round(now - self.start),
                "rtcm_bytes": self.bytes,
                "rtcm_messages": dict(sorted(self.counts.items())),
                "last_rtcm_age_s": round(now - self.last_rtcm, 1) if self.last_rtcm else None,
                "last_1005_age_s": round(now - self.last_1005, 1) if self.last_1005 else None,
                "base_ready": bool(self.last_1005 and now - self.last_1005 < 15),
                "survey_in": self.last_survey,
                "tcp_clients": self.tcp.count(),
                "ntrip_connected": self.publisher.published if self.publisher else None,
                "ntrip_dropped_frames": self.publisher.drop_count if self.publisher else None,
            }


def run_status_http(state, port, stop):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path not in ("/healthz", "/status"):
                self.send_error(404)
                return
            info = state.status()
            healthy = info["base_ready"]
            code = 200 if self.path == "/status" or healthy else 503
            payload = json.dumps(info, indent=2).encode() + b"\n"
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, fmt, *args):
            return

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    server.timeout = 0.5
    try:
        while not stop.is_set():
            server.handle_request()
    finally:
        server.server_close()


def serve(args):
    stop = threading.Event()
    def stop_signal(signum, frame):
        LOG.info("Stopping on signal %s", signum)
        stop.set()
    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGINT, stop_signal)

    publisher = None
    ntrip_host = os.getenv("NTRIP_HOST", "").strip()
    if ntrip_host:
        mount = os.getenv("NTRIP_MOUNT", "").strip().lstrip("/")
        pwd = os.getenv("NTRIP_PASSWORD", "")
        if not mount or not pwd or any(c in mount for c in " /\\\r\n") or "\r" in pwd or "\n" in pwd:
            raise ValueError("NTRIP_HOST requires valid NTRIP_MOUNT and NTRIP_PASSWORD")
        try:
            mount.encode("ascii")
            pwd.encode("ascii")
        except UnicodeEncodeError as exc:
            raise ValueError("NTRIP v1 SOURCE mount and password must be ASCII") from exc
        publisher = NtripPublisher(
            ntrip_host, int(os.getenv("NTRIP_PORT", "2101")), mount, pwd,
            os.getenv("NTRIP_TLS", "false").lower() in ("true", "1", "yes"),
        )

    tcp = RTCMTCPServer(args.tcp_port)
    state = RuntimeState(tcp, publisher)
    threads = [
        threading.Thread(target=tcp.accept_loop, args=(stop,), daemon=True),
        threading.Thread(target=run_status_http, args=(state, args.status_port, stop), daemon=True),
    ]
    if publisher:
        threads.append(threading.Thread(target=publisher.loop, args=(stop,), daemon=True))
    for thread in threads:
        thread.start()
    LOG.info("RTCM3 TCP listening on port %d; status on port %d", args.tcp_port, args.status_port)
    try:
        with open_port(args.device, args.baud) as port:
            # POLLED NAV-SVIN is retained even when the USB stream contains RTCM.
            port.write(ubx_frame(0x01, 0x3B))
            parser = StreamParser()
            next_poll = time.monotonic() + 10
            next_report = time.monotonic() + 30
            while not stop.is_set():
                block = port.read(4096)
                for packet in parser.feed(block):
                    state.process(packet)
                if time.monotonic() >= next_poll:
                    port.write(ubx_frame(0x01, 0x3B))
                    next_poll = time.monotonic() + 10
                if time.monotonic() >= next_report:
                    info = state.status()
                    LOG.info("base_ready=%s, RTCM messages=%s, TCP clients=%d, NTRIP=%s", info["base_ready"], info["rtcm_messages"], info["tcp_clients"], info["ntrip_connected"])
                    next_report = time.monotonic() + 30
    finally:
        stop.set()
        tcp.close()
    return 0


class IncomingTCPSerial:
    """Serial-like stream for Docker Desktop hosts without USB passthrough.

    The macOS host owns the real /dev/cu.* port and initiates a TCP connection
    to this listener through a *loopback-only* Docker published port. This is
    deliberately not an unauthenticated TCP server exposed on the Mac's LAN.

    The bridge forwards unmodified bytes in both directions; UBX configuration,
    surveys, RTCM and the existing parser therefore remain unchanged.
    """

    def __init__(self, url: str, accept_timeout: float = 30):
        parsed = urlsplit(url)
        if (parsed.scheme != "listen" or parsed.hostname not in ("0.0.0.0", "127.0.0.1")
                or not parsed.port or parsed.path or parsed.query or parsed.fragment):
            raise ValueError("Expected listen://0.0.0.0:PORT")

        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection = None
        try:
            self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.listener.bind((parsed.hostname, parsed.port))
            self.listener.listen(1)
            LOG.info("Waiting for macOS USB serial bridge on TCP port %d", parsed.port)
            end = time.monotonic() + accept_timeout
            while self.connection is None:
                remaining = end - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        "macOS serial bridge not connected: run "
                        "'python scripts/mac_serial_bridge.py --port /dev/cu.usbmodem...' "
                        "on the Mac, then retry"
                    )
                self.listener.settimeout(min(1.0, remaining))
                try:
                    self.connection, address = self.listener.accept()
                except socket.timeout:
                    continue
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.connection.settimeout(0.2)
            LOG.info("macOS USB serial bridge connected from %s", address[0])
        except BaseException:
            self.close()
            raise
        finally:
            # One bridge owns a receiver at a time. Close listener after accept.
            self.listener.close()

    def read(self, size: int = 1) -> bytes:
        try:
            data = self.connection.recv(size)
        except socket.timeout:
            return b""
        except OSError as exc:
            raise OSError(f"macOS USB bridge read failed: {exc}") from exc
        if not data:
            raise OSError("macOS USB bridge disconnected; restart it or reconnect the device")
        return data

    def write(self, data: bytes) -> int:
        try:
            self.connection.sendall(data)
        except OSError as exc:
            raise OSError(f"macOS USB bridge write failed: {exc}") from exc
        return len(data)

    def reset_input_buffer(self) -> None:
        # GNSS continues emitting asynchronous RTCM while the app polls UBX.
        # Drop already queued bytes, as Serial.reset_input_buffer() would.
        try:
            while select.select([self.connection], [], [], 0)[0]:
                data = self.connection.recv(4096)
                if not data:
                    raise OSError("macOS USB bridge disconnected")
        except socket.timeout:
            pass

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None
        self.listener.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def open_port(path, baud):
    if path.startswith("listen://"):
        return IncomingTCPSerial(path, accept_timeout=float(os.getenv("GNSS_BRIDGE_WAIT_S", "30")))
    if serial is None:
        raise RuntimeError("pyserial not available; build or install requirements")
    return serial.Serial(path, baudrate=baud, timeout=0.2, write_timeout=3, exclusive=True)


def mode_label(code: int) -> str:
    return {0: "disabled", 1: "survey-in", 2: "fixed"}.get(code, f"unknown({code})")


def command(args):
    if args.action == "serve":
        return serve(args)
    with open_port(args.device, args.baud) as port:
        dev = GNSSDevice(port)
        if args.action == "probe":
            print(json.dumps(dev.mon_ver(), indent=2))
        elif args.action in ("survey", "fixed"):
            configure_device(dev, args)
        elif args.action == "status":
            config = dev.valget(["CFG-TMODE-MODE"])
            result = {"mode": mode_label(config["CFG-TMODE-MODE"]), "mode_code": config["CFG-TMODE-MODE"]}
            try:
                result["survey_in"] = dev.survey_status()
            except TimeoutError:
                result["survey_in"] = None
            print(json.dumps(result, indent=2))
        else:
            raise ValueError(f"Unknown command {args.action}")
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--device", default=os.getenv("GNSS_DEVICE", "/dev/gnss"))
    parser.add_argument("--baud", type=int, default=int(os.getenv("GNSS_BAUD", "38400")))
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("probe", help="Read UBX-MON-VER to confirm USB communication")
    sub.add_parser("status", help="Poll base mode and UBX-NAV-SVIN (stop stream first)")
    survey = sub.add_parser("survey", help="Configure stationary survey-in base")
    survey.add_argument("--duration", type=int, default=600, help="minimum survey duration, seconds")
    survey.add_argument("--accuracy-m", type=float, default=5.0, help="estimated survey-in threshold, NOT absolute accuracy")
    fixed = sub.add_parser("fixed", help="Configure fixed, externally surveyed ARP ECEF coordinates")
    for axis in "xyz":
        fixed.add_argument(f"--ecef-{axis}", type=float, required=True, help="ARP coordinate (meters, ECEF)")
    fixed.add_argument("--accuracy-m", type=float, required=True, help="surveyed 3D coordinate accuracy in meters")
    for p in (survey, fixed):
        p.add_argument("--uart2", action="store_true", help="also output RTCM on UART2 (hardware wiring required)")
        p.add_argument("--uart2-baud", type=int, default=115200)
    run = sub.add_parser("serve", help="Stream CRC-checked RTCM3 over TCP, optionally push to NTRIP caster")
    run.add_argument("--tcp-port", type=int, default=int(os.getenv("TCP_PORT", "2102")))
    run.add_argument("--status-port", type=int, default=int(os.getenv("STATUS_PORT", "8080")))
    return parser


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = build_parser().parse_args()
    if args.action == "survey" and args.duration <= 0:
        raise SystemExit("--duration must be positive")
    try:
        return command(args)
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        LOG.error("%s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""macOS-side serial bridge for SparkFun ZED-F9P + Docker Desktop.

USB serial devices are visible to the macOS host, not to Docker Desktop's Linux
VM. This script owns the /dev/cu.* device and makes an *outbound* TCP connection
to the container through a localhost-only published port. It does not listen on
any macOS network interface or require privileged containers / USB/IP.

Use:
  python3 -m pip install pyserial==3.5
  python3 scripts/mac_serial_bridge.py --list
  python3 scripts/mac_serial_bridge.py --port /dev/cu.usbmodemXXXX
"""
from __future__ import annotations

import argparse
import logging
import os
import select
import socket
import sys
import time

LOG = logging.getLogger("f9p-mac-bridge")


def forward_serial(ser, conn: socket.socket) -> tuple[int, int]:
    """Copy unmodified GNSS/RTCM/UBX bytes bidirectionally until disconnected.

    The caller controls the physical serial baud rate and USB port ownership.
    select() works for pyserial's macOS POSIX serial descriptors.
    """
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.settimeout(3)
    serial_fd = ser.fileno()
    to_container = to_receiver = 0
    while True:
        readable, _, _ = select.select([serial_fd, conn], [], [], 0.5)
        if conn in readable:
            payload = conn.recv(4096)
            if not payload:
                return to_container, to_receiver
            written = ser.write(payload)
            if written != len(payload):
                raise OSError("short serial write")
            to_receiver += written
        if serial_fd in readable:
            payload = ser.read(min(max(ser.in_waiting, 1), 4096))
            if not payload:
                raise OSError("GNSS serial port disconnected")
            conn.sendall(payload)
            to_container += len(payload)


def list_devices():
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise RuntimeError("Install pyserial first: python3 -m pip install pyserial==3.5") from exc
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports detected. Connect the SparkFun board with a USB-C data cable.")
    for port in ports:
        print(f"{port.device:<30} {port.description}  {port.hwid}")
    print("Use the /dev/cu.* (call-out) device, not /dev/tty.*.")


def run_bridge(device: str, baud: int, target_host: str, target_port: int, retry_s: float):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("Install pyserial first: python3 -m pip install pyserial==3.5") from exc
    last_failure = None
    while True:
        try:
            # USB opens on macOS. The container is never granted hardware access.
            with serial.Serial(device, baudrate=baud, timeout=0,
                               write_timeout=3, exclusive=True) as ser:
                LOG.info("Opened GNSS serial port %s (baud=%d)", device, baud)
                while True:
                    try:
                        with socket.create_connection((target_host, target_port), timeout=3) as conn:
                            LOG.info("Connected to Docker GNSS listener at %s:%d", target_host, target_port)
                            last_failure = None
                            ser.reset_input_buffer()
                            uplink, downlink = forward_serial(ser, conn)
                            LOG.info("Docker session ended (GNSS→Docker %d bytes, Docker→GNSS %d bytes)",
                                     uplink, downlink)
                    except (OSError, TimeoutError) as exc:
                        message = str(exc)
                        if message != last_failure:
                            LOG.warning("Waiting for container, or connection lost: %s", message)
                            last_failure = message
                    time.sleep(retry_s)
        except (OSError, serial.SerialException) as exc:
            message = str(exc)
            if message != last_failure:
                LOG.error("Cannot use %s: %s; check /dev/cu.* and cable", device, message)
                last_failure = message
            time.sleep(retry_s)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="list host macOS serial devices")
    parser.add_argument("--port", help="SparkFun serial device on macOS: /dev/cu.usbmodem...")
    parser.add_argument("--baud", type=int, default=int(os.getenv("GNSS_BAUD", "38400")))
    parser.add_argument("--host", default="127.0.0.1", help="container published port host")
    parser.add_argument("--tcp-port", type=int, default=int(os.getenv("GNSS_BRIDGE_PORT", "45321")))
    parser.add_argument("--retry", type=float, default=1.0, help="reconnection delay in seconds")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if args.list:
        list_devices()
        return 0
    if not args.port:
        parser.error("--port is required unless --list is provided")
    if not args.port.startswith("/dev/cu."):
        parser.error("on macOS use a /dev/cu.* serial device, not /dev/tty.*")
    if not (1 <= args.tcp_port <= 65535 and args.retry > 0 and args.baud > 0):
        parser.error("port and baud must be positive; --tcp-port <=65535; --retry >0")
    try:
        run_bridge(args.port, args.baud, args.host, args.tcp_port, args.retry)
    except KeyboardInterrupt:
        LOG.info("Bridge stopped")
    except RuntimeError as exc:
        LOG.error("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

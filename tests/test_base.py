"""Offline tests; no GNSS hardware, Docker, or pyserial required."""
import struct
import socket
import threading
import unittest
from types import SimpleNamespace
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gnss_base import (
    Packet, GNSSDevice, KEYS, RTCM_RATES, RuntimeState, StreamParser,
    accuracy_units, base_config, configure_device, crc24q, decode_svin,
    pack_value, split_ecef, ubx_checksum, ubx_frame, unpack_value, NtripPublisher,
)


class FakePort:
    def __init__(self, reject_flash=False):
        self.rx = bytearray()
        self.settings = {KEYS['CFG-TMODE-MODE']: 0}
        self.reject_flash = reject_flash
        self.sent = []

    def reset_input_buffer(self):
        self.rx.clear()

    def read(self, n):
        if not self.rx:
            return b''
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def write(self, data):
        self.sent.append(data)
        assert data[:2] == b'\xb5\x62'
        assert ubx_checksum(data[2:-2]) == data[-2:]
        cls, mid, size = struct.unpack_from('<BBH', data, 2)
        body = data[6:6+size]
        if (cls, mid) == (0x0A, 0x04):
            p = b'HPG 1.51'.ljust(30, b'\x00') + b'00080000'.ljust(10, b'\x00')
            p += b'MOD=ZED-F9P'.ljust(30, b'\x00')
            p += b'PROTVER=27.50'.ljust(30, b'\x00')
            self.rx.extend(ubx_frame(cls, mid, p))
        elif (cls, mid) == (0x06, 0x8A):
            if self.reject_flash and (body[1] & 4):
                self.rx.extend(ubx_frame(0x05, 0x00, b'\x06\x8a'))
            else:
                cursor = 4
                while cursor < len(body):
                    key = int.from_bytes(body[cursor:cursor+4], 'little')
                    cursor += 4
                    value, size = unpack_value(key, body[cursor:])
                    cursor += size
                    self.settings[key] = value
                self.rx.extend(ubx_frame(0x05, 0x01, b'\x06\x8a'))
        elif (cls, mid) == (0x06, 0x8B):
            p = bytearray(b'\x01\x00\x00\x00')
            for offset in range(4, len(body), 4):
                key = int.from_bytes(body[offset:offset+4], 'little')
                if key in self.settings:
                    p.extend(struct.pack('<I', key))
                    p.extend(pack_value(key, self.settings[key]))
            self.rx.extend(ubx_frame(cls, mid, bytes(p)))
        elif (cls, mid) == (0x01, 0x3B):
            p = bytearray(40)
            struct.pack_into('<I', p, 8, 600)
            struct.pack_into('<i', p, 12, 398672111)
            struct.pack_into('<b', p, 24, -23)
            struct.pack_into('<I', p, 28, 28750)
            struct.pack_into('<I', p, 32, 601)
            p[36:38] = b'\x01\x00'
            self.rx.extend(ubx_frame(cls, mid, bytes(p)))
        else:
            raise AssertionError(f'Unexpected command: {(cls,mid)}')
        return len(data)


class TestUBXProtocol(unittest.TestCase):
    def test_ubx_checksum_standard(self):
        # UBX-MON-VER poll: class=0A id=04, empty payload.
        self.assertEqual(ubx_frame(0x0A, 0x04), bytes.fromhex('b5620a0400000e34'))

    def test_crc24q_reference(self):
        self.assertEqual(crc24q(b'123456789'), 0xCDE703)

    def test_stream_fragmentation_interleaved_with_noise(self):
        payload = bytes([0x43, 0x20, 0x19, 0xAB])  # RTCM type 1074
        rtcm_wo_crc = bytes([0xD3, 0x00, len(payload)]) + payload
        rtcm = rtcm_wo_crc + crc24q(rtcm_wo_crc).to_bytes(3, 'big')
        ubx = ubx_frame(1, 0x3B, bytes(40))
        parser = StreamParser()
        stream = b'$GNGGA,12345*00\r\n' + ubx + rtcm + b'\xD3\x00\x02bad' + rtcm
        result = []
        for i in range(0, len(stream), 3):
            result += parser.feed(stream[i:i+3])
        self.assertEqual([p.protocol for p in result], ['ubx', 'rtcm', 'rtcm'])
        self.assertEqual([p.type for p in result if p.protocol == 'rtcm'], [1074, 1074])
        self.assertEqual([p.raw for p in result if p.protocol == 'rtcm'], [rtcm, rtcm])

    def test_key_widths_and_signedness(self):
        for axis in 'XYZ':
            for value in (-137827112, 0, 317112345):
                key = KEYS[f'CFG-TMODE-ECEF_{axis}']
                encoded = pack_value(key, value)
                self.assertEqual(unpack_value(key, encoded)[0], value)
            k = KEYS[f'CFG-TMODE-ECEF_{axis}_HP']
            self.assertEqual(unpack_value(k, pack_value(k, -49))[0], -49)
        self.assertEqual(len(pack_value(KEYS['CFG-RATE-MEAS'], 1000)), 2)

    def test_coordinate_roundtrip(self):
        for value in (3986721.11873, -392293.22814, 0.0, -0.0098, 6378137.0):
            coarse, hp = split_ecef(value)
            self.assertAlmostEqual(coarse / 100 + hp / 10000, value, delta=0.000051)
            self.assertLessEqual(abs(hp), 50)
        with self.assertRaises(ValueError):
            split_ecef(23_000_000)

    def test_accuracy_units(self):
        self.assertEqual(accuracy_units(5), 50_000)
        self.assertEqual(accuracy_units(0.05), 500)
        self.assertRaises(ValueError, accuracy_units, 0)

    def test_svin_offsets(self):
        dev = GNSSDevice(FakePort())
        svin = dev.survey_status()
        self.assertTrue(svin['valid'])
        self.assertFalse(svin['active'])
        self.assertEqual(svin['duration_s'], 600)
        self.assertAlmostEqual(svin['ecef_m'][0], 3986721.1077)
        self.assertAlmostEqual(svin['accuracy_m_estimate'], 2.875)


class TestConfiguration(unittest.TestCase):
    def test_mon_ver(self):
        self.assertIn('ZED-F9P', ''.join(GNSSDevice(FakePort()).mon_ver()['extensions']))

    def test_base_message_rates(self):
        cfg = base_config()
        for typ, rate in RTCM_RATES.items():
            self.assertEqual(cfg[f'CFG-MSGOUT-RTCM_3X_TYPE{typ}_USB'], rate)
        self.assertEqual(cfg['CFG-USBOUTPROT-RTCM3X'], 1)
        self.assertEqual(cfg['CFG-USBOUTPROT-NMEA'], 0)
        self.assertEqual(cfg['CFG-USBOUTPROT-UBX'], 1)

    def test_survey_configuration_with_readback(self):
        port = FakePort()
        dev = GNSSDevice(port, timeout=0.2)
        args = SimpleNamespace(action='survey', duration=900, accuracy_m=5,
                               uart2=False, uart2_baud=115200)
        configure_device(dev, args)
        self.assertEqual(port.settings[KEYS['CFG-TMODE-MODE']], 1)
        self.assertEqual(port.settings[KEYS['CFG-TMODE-SVIN_MIN_DUR']], 900)
        self.assertEqual(port.settings[KEYS['CFG-TMODE-SVIN_ACC_LIMIT']], 50000)
        self.assertEqual(port.settings[KEYS['CFG-MSGOUT-RTCM_3X_TYPE1005_USB']], 5)

    def test_fixed_configuration_with_negatives(self):
        port = FakePort()
        dev = GNSSDevice(port, timeout=0.2)
        args = SimpleNamespace(action='fixed', accuracy_m=0.05,
                               ecef_x=-2360000.1234, ecef_y=465430.1113,
                               ecef_z=5400123.4567, uart2=True, uart2_baud=115200)
        configure_device(dev, args)
        self.assertEqual(port.settings[KEYS['CFG-TMODE-MODE']], 2)
        self.assertEqual(port.settings[KEYS['CFG-TMODE-POS_TYPE']], 0)
        self.assertEqual(port.settings[KEYS['CFG-TMODE-FIXED_POS_ACC']], 500)
        self.assertEqual(port.settings[KEYS['CFG-UART2-BAUDRATE']], 115200)
        for axis, expected in zip('XYZ', (-2360000.1234, 465430.1113, 5400123.4567)):
            cm = port.settings[KEYS[f'CFG-TMODE-ECEF_{axis}']]
            hp = port.settings[KEYS[f'CFG-TMODE-ECEF_{axis}_HP']]
            self.assertAlmostEqual(cm/100 + hp/10000, expected, delta=0.00005)

    def test_fallback_to_bbr_if_flash_unavailable(self):
        port = FakePort(reject_flash=True)
        dev = GNSSDevice(port, timeout=0.2)
        args = SimpleNamespace(action='survey', duration=600, accuracy_m=5,
                               uart2=False, uart2_baud=115200)
        configure_device(dev, args)
        self.assertEqual(port.settings[KEYS['CFG-TMODE-MODE']], 1)
        vals = [request[7] for request in port.sent if request[2:4] == b'\x06\x8a']
        self.assertIn(7, vals)
        self.assertIn(3, vals)


class TestNTRIP(unittest.TestCase):
    def test_ntrip_v1_source_handshake_localhost(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('127.0.0.1', 0))
        server.listen(1)
        server.settimeout(3)
        port = server.getsockname()[1]
        received = {}

        def caster():
            try:
                with server.accept()[0] as conn:
                    conn.settimeout(3)
                    line = b''
                    while not line.endswith(b'\r\n\r\n'):
                        part = conn.recv(1)
                        if not part:
                            break
                        line += part
                    received['handshake'] = line
                    conn.sendall(b'ICY 200 OK\r\n')
                    received['frame'] = conn.recv(100)
            finally:
                server.close()

        t = threading.Thread(target=caster)
        t.start()
        publisher = NtripPublisher('127.0.0.1', port, 'TEST', 'secret')
        with publisher._connect() as client:
            client.sendall(b'\xd3\x00\x03abc')
        t.join(timeout=3)
        self.assertFalse(t.is_alive())
        self.assertTrue(received['handshake'].startswith(b'SOURCE secret /TEST\r\n'))
        self.assertEqual(received['frame'], b'\xd3\x00\x03abc')


class DummyTCP:
    def __init__(self):
        self.sent = []

    def broadcast(self, frame):
        self.sent.append(frame)

    def count(self):
        return 0


class TestRuntime(unittest.TestCase):
    def test_status_ready_only_after_1005(self):
        tcp = DummyTCP()
        state = RuntimeState(tcp)
        msg = Packet('rtcm', 1074, b'', b'frame')
        state.process(msg)
        self.assertFalse(state.status()['base_ready'])
        state.process(Packet('rtcm', 1005, b'', b'base'))
        self.assertTrue(state.status()['base_ready'])
        self.assertEqual(tcp.sent, [b'frame', b'base'])


if __name__ == '__main__':
    unittest.main()

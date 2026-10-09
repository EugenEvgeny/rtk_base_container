# SparkFun ZED-F9P RTK base station in Docker

A Docker Compose deployment for the **SparkFun GPS-RTK-SMA Breakout – ZED-F9P (Qwiic), GPS-16481**, including **ZED-F9P-05B-00** modules, on **Linux or macOS (Docker Desktop; Intel/Apple silicon)**.

One **USB-C data cable** powers the SparkFun breakout **and** carries GNSS/RTCM data to the computer. No USB-to-UART adapter, Qwiic controller, or external 5 V supply is needed. Connect a compatible **active L1/L2 GNSS antenna** to the SMA connector, securely install it with a clear sky view and suitable ground plane, and do not move it after surveying.

The project deliberately has **no receiver credentials, hard-coded coordinates or radio settings**. Configure once, then run the service; there is **no receiver reset or rewrite on every container restart**.

## Features

- `probe`: read the receiver's UBX-MON-VER version information.
- `survey`: configure stationary RTK base with configurable **minimum survey-in duration** and **estimated-accuracy threshold**.
- `fixed`: configure base using **externally surveyed antenna reference point (ARP) ECEF coordinates** in **meters**, including high-precision 0.1-mm components.
- `status`: poll base mode and UBX-NAV-SVIN survey status.
- `serve`: publish only **CRC-verified RTCM3 binary frames** on a raw TCP socket (port **2102**) and provide JSON status/health endpoints (port **8080**); optional upstream **NTRIP v1 SOURCE** publishing to an **existing** caster.
- Optional `--uart2` configures UART2 at 115200 and duplicates RTCM3 output for an external radio. Not required for USB-only operation.
- Configures six usual RTCM MSM4 messages: **1005 every 5 s; 1074, 1084, 1094, 1124 every 1 s; 1230 every 5 s**. Disables previously enabled competing MSM7 observations (1077, 1087, 1097, 1127) and RTCM 1006 on configured ports.
- Receives acknowledgements for configuration writes and checks configuration readback before enabling the base mode.
- Supports **Docker Desktop for Mac** using a local-only, bidirectional serial-to-TCP bridge. No USB/IP or privileged container needed; the same GNSS command-line interface runs on Linux and macOS.

**Not a built-in NTRIP caster:** The TCP port is **raw RTCM, not NTRIP HTTP**. For NTRIP rover clients, configure `NTRIP_*` to publish to a separate NTRIP caster that you operate or have permission to use.

## macOS / Docker Desktop setup (Intel and Apple silicon)

Docker Desktop cannot directly map a macOS `/dev/cu.*` USB serial device to a Linux container with Compose's `devices:` directive. **Do not use `compose.yaml` on macOS.** Instead, this project supplies a `compose.mac.yaml` and a small Python program that runs **on the Mac**:

```text
SparkFun ZED-F9P --USB-C--> Mac /dev/cu.usbmodem...
     Mac host script: scripts/mac_serial_bridge.py
            outbound TCP: 127.0.0.1:45321
            Docker Desktop port forwarding (localhost only)
     Docker container: GNSS config, survey-in, RTCM, status API
            |-- 127.0.0.1:2102  raw RTCM3 over TCP
            `-- 127.0.0.1:8080  /status and /healthz
```

The **Mac** owns the physical USB serial port; the container never attempts USB device passthrough. The Mac bridge **initiates an outbound connection** to the Docker container, so it does not expose an unauthenticated serial server on the Mac's Wi-Fi/Ethernet interface. The port mapping on Docker is explicitly bound to `127.0.0.1`.

### 1. Install dependencies on the Mac

Install and start [Docker Desktop for Mac](https://docs.docker.com/desktop/setup/install/mac-install/). Python 3 with `venv` support is also needed on macOS **outside** Docker.

From the downloaded project folder, in a regular **macOS Terminal**:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install 'pyserial==3.5'
```

Connect the SparkFun breakout to the Mac using a USB-C **data** cable, and connect its active antenna. Identify the macOS serial device:

```bash
python3 scripts/mac_serial_bridge.py --list
ls /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null
```

Look for the ZED-F9P; macOS typically enumerates its native USB serial interface as `/dev/cu.usbmodemXXXX`. **Use the actual `/dev/cu.*` path**, not a Linux `/dev/ttyACM0` path or macOS `/dev/tty.*` call-in device.

### 2. Configure the macOS Compose deployment

In the project folder:

```bash
cp .env.mac.example .env
docker compose -f compose.mac.yaml build
```

If `.env` already exists, keep it or back it up before replacing it. macOS does **not** use `GNSS_HOST_DEVICE` or `GNSS_DEVICE_GID`; those settings are only for Linux. The defaults use local TCP port **45321** for the Mac-to-container bridge, raw RTCM **2102**, and HTTP status **8080**.

The project uses the standard multi-architecture Python Docker image and does not require an explicit `platform: linux/amd64` setting for Apple silicon.
If you change `GNSS_BRIDGE_PORT` in `.env`, also pass the same number as `--tcp-port` to the host bridge in the next step. If you change `GNSS_BAUD`, also pass `--baud` to the host bridge: the `.env` file configures Docker, but is not automatically loaded by the host-side Python script.

### 3. Start the host serial bridge in Terminal A

Replace the example USB name with the actual port from step 1:

```bash
source .venv/bin/activate
python3 scripts/mac_serial_bridge.py --port /dev/cu.usbmodemXXXX
```

**Leave this terminal running.** At first it may log a connection-refused warning: this is expected until a container is running. The script automatically retries and reconnects after each one-off `probe`, `survey`, or `status` container exits. If the physical USB disconnects, it also retries reopening the specified device path.

### 4. Probe and configure the ZED-F9P in Terminal B

All one-off commands **must** use `--service-ports`: this publishes the local TCP listener so the Mac host bridge can connect. The running `base` service must be stopped first to avoid port conflicts.

```bash
# In the project directory; the Mac bridge must remain running in Terminal A.
docker compose -f compose.mac.yaml run --rm --service-ports base probe

# Configure stationary RTK base; 10-minute minimum survey-in.
docker compose -f compose.mac.yaml run --rm --service-ports base survey \
  --duration 600 --accuracy-m 5

# Confirm mode_code=1 (survey-in) after configuration.
docker compose -f compose.mac.yaml run --rm --service-ports base status
```

The `probe` output should contain `MOD=ZED-F9P` and `PROTVER=27.50` for your reported HPG 1.51 firmware. The `survey` command writes configuration and verifies readback; if it fails partway through, the receiver may still have `CFG-TMODE-MODE=0`, so inspect the complete error output rather than assuming survey-in started.

### 5. Run continuously and monitor

```bash
docker compose -f compose.mac.yaml up -d base
docker compose -f compose.mac.yaml logs -f base
```

The status endpoint and correction stream are then accessible from the **Mac**:

```bash
curl -s http://127.0.0.1:8080/status
curl -i http://127.0.0.1:8080/healthz
```

When survey-in has finished (`survey_in.valid=true` and `survey_in.active=false`), check that RTCM 1005/1074/1084/1094/1124/1230 message counts and `rtcm_bytes` are increasing. `base_ready=true` means an RTCM 1005 message has arrived recently, **not** that your absolute coordinates have centimeter accuracy.

Before running further configuration or a one-off status command, **stop the service** (the bridge will reconnect):

```bash
docker compose -f compose.mac.yaml stop base
docker compose -f compose.mac.yaml run --rm --service-ports base status
docker compose -f compose.mac.yaml up -d base
```

For a permanent surveyed installation, use fixed coordinates with the same syntax as the Linux `fixed` command below but prepend `-f compose.mac.yaml` and include `--service-ports`. Do not guess ECEF coordinates.

### macOS diagnostics

| Symptom | Solution |
|---|---|
| `/dev/cu.usbmodem*` not present | Check USB-C **data** cable, board power, USB hub and `mac_serial_bridge.py --list` output. |
| `Permission denied` or `Resource busy` opening `/dev/cu.*` | Close other serial applications, such as screen, IDE serial monitors and GNSS tools. No Linux `dialout` group applies on macOS. |
| Mac bridge prints `Connection refused` | Start a Compose `run --service-ports` command or `up -d base`. Check both use **compose.mac.yaml** and the same bridge TCP port. |
| `macOS serial bridge not connected` in Docker logs | Check that the host bridge is running and the Compose port `127.0.0.1:45321:45321` is published. |
| Compose reports `port is already allocated` | Stop the `base` service before invoking one-off commands, or find the application already using port 45321, 2102 or 8080. |
| Bridge connects but `mode_code=0` | Re-run the `survey` configuration and inspect configuration write/readback errors. |
| Survey remains at zero observations | Check that the receiver has a valid GNSS fix, antenna placement and open-sky reception. |
| USB unplugged and `/dev/cu.*` name changes | Run `--list` again, then restart the Mac bridge with the new device name. |

**Security:** Do not publish the bridge's port to `0.0.0.0` or a LAN IP: the serial transport provides direct read/write access to receiver configuration, and has no authentication or encryption. The supplied `compose.mac.yaml` binds this port to the Mac's loopback interface only. Exposing the separate RTCM *output* port 2102 is optional and should be restricted to trusted networks. Do not use the `--service-ports` one-off option while the background service is running.

## Linux prerequisites

- Linux host: Ubuntu, Debian, Raspberry Pi OS or similar with **Docker Engine** and **Docker Compose v2**. The commands in the remainder of this section use the **Linux** `compose.yaml` (not the macOS Compose file).
- SparkFun breakout and an active L1/L2 antenna connected to SMA.
- USB-C **data** cable from breakout to the Linux host (USB supplies 5 V to the breakout's onboard regulator).
- Ensure no other process owns the receiver (`gpsd`, ModemManager, serial consoles, u-center, etc.).

## Linux first-time setup

### 1. Identify the USB device

Plug in the board and run on your Linux **host**:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

The native USB connection normally appears as `/dev/ttyACM0`. Prefer a stable full path under `/dev/serial/by-id/` if present. Do not assume that another `/dev/ttyACM0` device is your F9P.

Example using the actual full device path (replace it with yours):

```bash
GNSS_HOST_DEVICE=/dev/serial/by-id/usb-u-blox_...  # fill in actual path
readlink -f "$GNSS_HOST_DEVICE"
stat -Lc '%g' "$GNSS_HOST_DEVICE"
```

The last command outputs the **numeric GID** of the device, which Docker needs for non-root serial access.

If you have no stable by-id path, set `GNSS_HOST_DEVICE=/dev/ttyACM0` in the next step, after verifying it is your receiver.

### 2. Create the environment file

```bash
cp .env.example .env
nano .env
```

Set these required values:

```dotenv
GNSS_HOST_DEVICE=/dev/serial/by-id/YOUR_ACTUAL_RECEIVER
GNSS_DEVICE_GID=20
```

Replace `20` with the **actual numeric GID** printed by `stat`. The default `TCP_BIND_IP=127.0.0.1` keeps RTCM local to the host; you can later bind the stream to the host's LAN address for a rover.

This project needs neither `privileged: true` nor host-network mode. The Compose file passes one USB device into `/dev/gnss` and grants the host device GID to the container's non-root user.

If the port is occupied, temporarily stop competing services on the **host**, for example:

```bash
sudo systemctl stop gpsd.socket gpsd.service 2>/dev/null || true
sudo systemctl stop ModemManager.service 2>/dev/null || true
```

The host may restart these services unless reconfigured to ignore this device.

### 3. Build image and probe receiver

```bash
docker compose build
docker compose run --rm base probe
```

Look for `HPG ...`, `PROTVER=...` and `MOD=ZED-F9P` in the response. The module's shipped firmware may differ from the version you previously observed. The script does **not** flash firmware.

### 4. Configure stationary base with survey-in

```bash
docker compose run --rm base survey --duration 600 --accuracy-m 5
```

The duration is **10 minutes minimum** and `5` is a **5-meter estimated survey-in threshold**. It is **not** a 5-m absolute accuracy guarantee and cannot establish centimeter-level absolute coordinates on its own. In a difficult antenna location survey-in can take longer or fail to converge.

The command sets and checks:

- `CFG-TMODE-MODE=0` during configuration, then `=1` (survey-in)
- `CFG-TMODE-SVIN_MIN_DUR=600`, `CFG-TMODE-SVIN_ACC_LIMIT=50000` (0.1-mm units)
- `CFG-RATE-MEAS=1000`, `CFG-RATE-NAV=1`
- USB output: UBX enabled, RTCM3 enabled, NMEA disabled
- USB RTCM 1005/1074/1084/1094/1124/1230 rates `5/1/1/1/1/5` navigation epochs, plus suppression of alternate MSM7 and 1006 outputs

The script tries saving RAM+BBR+Flash. If Flash is unsupported, it warns and retries RAM+BBR, then RAM-only. **RAM-only configurations will be lost on reboot**; BBR requires backup power, while Flash persistence depends on receiver hardware.

### 5. Check survey-in progress

```bash
docker compose run --rm base status
```

Example after a completed survey-in (values illustrative):

```json
{
  "mode": "survey-in",
  "mode_code": 1,
  "survey_in": {
    "duration_s": 639,
    "accuracy_m_estimate": 2.9,
    "observation_count": 640,
    "valid": true,
    "active": false,
    "ecef_m": [3986721.1077, 1013445.1234, 4881231.4567]
  }
}
```

Ready survey-in: `valid=true`, `active=false`. The reported ECEF mean is the **uncorrected surveyed mean**, not an externally surveyed centimeter-accurate reference point.

### 6. Start streaming continuously

```bash
docker compose up -d

docker compose logs -f base
```

The receiver is **not reconfigured** in `serve` mode, and the container does **not** itself power-cycle the USB device. If power is lost and the receiver is still in survey-in mode, it may need to perform survey-in again before RTCM base messages resume.

Check the status/health endpoint from the Linux host:

```bash
curl -s http://127.0.0.1:8080/status
curl -i http://127.0.0.1:8080/healthz
```

`base_ready=true` means a valid CRC-checked RTCM 1005 message has been received recently. `/healthz` returns HTTP **503** until 1005 is seen and then **200** while it remains fresh. This is a **stream-readiness indicator**, *not* proof of rover RTK FIX or absolute coordinate quality.

Example:

```json
{
  "rtcm_messages": {"1005": 10, "1074": 49, "1084": 48, "1094": 48, "1124": 48, "1230": 10},
  "last_1005_age_s": 1.5,
  "base_ready": true,
  "tcp_clients": 0,
  "ntrip_connected": null
}
```

The raw RTCM stream is at `127.0.0.1:2102` on the Linux host. A rover running on the **same host** can consume the stream via a raw TCP client. To make it available to a rover on your **local network**, edit `.env`:

```dotenv
TCP_BIND_IP=192.168.1.20
```

Use your **actual host LAN IP** (`ip -brief addr`); then run:

```bash
docker compose up -d --force-recreate
```

Your rover's correction client should then connect to `192.168.1.20:2102` as **raw RTCM3 over TCP**, not as an NTRIP mount point. Protect this socket behind a trusted network or VPN: it has **no built-in authentication**.

### 7. Optional: publish to an existing NTRIP caster

Edit `.env`:

```dotenv
NTRIP_HOST=caster.example.org
NTRIP_PORT=2101
NTRIP_MOUNT=MYBASE
NTRIP_PASSWORD=YOUR_CASTER_SOURCE_PASSWORD
NTRIP_TLS=false
```

Restart:

```bash
docker compose up -d --force-recreate
```

This uses the **NTRIP v1 `SOURCE`** upload protocol, not NTRIP v2 POST. Confirm your caster accepts source-password publishing; many public caster services do not accept arbitrary uploads. The host and mount are illustrative. Set `NTRIP_TLS=true` only if your caster supports TLS on that port. Credentials live in the `.env` file, not in the image; **keep `.env` private** and remember Docker may expose environment variables to host administrators.

Only CRC-validated RTCM frames are sent upstream. Failure of the upstream caster does **not** prevent local TCP streaming; the process retries the caster periodically.

## Permanent station: surveyed ECEF coordinates

If you have coordinates of the installed **antenna reference point (ARP)** from a reliable geodetic survey or appropriately corrected observation, prefer fixed mode:

1. Stop the running service to release the USB port:

   ```bash
   docker compose stop base
   ```

2. Export your **actual surveyed ARP ECEF coordinates in meters**, and their surveyed 3D coordinate uncertainty, as shell variables (`SURVEYED_X_M`, `SURVEYED_Y_M`, `SURVEYED_Z_M`, `SURVEY_ACCURACY_M`). Then execute:

   ```bash
   docker compose run --rm base fixed \
     --ecef-x "$SURVEYED_X_M" \
     --ecef-y "$SURVEYED_Y_M" \
     --ecef-z "$SURVEYED_Z_M" \
     --accuracy-m "$SURVEY_ACCURACY_M"
   ```

   These variables must contain real numeric survey results; no coordinates are provided by this project.

3. Verify and restart:

   ```bash
   docker compose run --rm base status
   docker compose up -d
   ```

`--accuracy-m` in fixed mode records your *known coordinate accuracy*, not an accuracy target for the receiver to achieve. Never invent the base coordinates or set an unjustifiably optimistic accuracy. Ensure ECEF coordinates use a consistent GNSS reference frame and epoch. Any base-coordinate error propagates to absolute rover coordinates.

## Optional UART2 corrections to a radio

Only if a **3.3 V logic UART radio** is physically connected to the board's broken-out UART2 signals:

```bash
docker compose stop base
docker compose run --rm base survey --duration 600 --accuracy-m 5 --uart2 --uart2-baud 115200
docker compose up -d
```

This enables the same RTCM messages on UART2. Use the board's **silkscreened** TXD2 and GND pads, not the internal module package pad numbers. Match the radio serial baud rate; do **not** apply RS-232 voltage levels to 3.3 V UART pins. Native USB streaming requires none of this wiring.

## Diagnostics and limitations

| Symptom | Check |
|---|---|
| `Permission denied: /dev/gnss` | `GNSS_DEVICE_GID` must match `stat -Lc '%g' /dev/ttyACM0`; check Compose `group_add`, unplug/replug. |
| `/dev/gnss` not found | Verify board/cable is plugged into host, `dmesg`, `lsusb`, `ls /dev/serial/by-id/`. |
| `Device or resource busy` | Only one application can open the USB serial device; `docker compose stop base` before `probe`, `survey`, `fixed`, or `status`. Stop gpsd/ModemManager or other clients. |
| Survey stays `active=true` | Need antenna open to sky; 5-m threshold may take longer than 600 s if multipath is severe. |
| `base_ready=false` | Wait for survey-in; check correct USB port and valid RTCM 1005 output. |
| TCP client connects but no rover FIX | Confirm receiver has a solution, GPS+L2 antenna signals, station coordinate, rover constellation support, correction latency, and RTCM messages. |
| NTRIP source rejected | Check mount/source password, caster v1 SOURCE compatibility, firewall, TLS setting. |

`status` and `probe` **cannot** run concurrently with `serve` on the same serial device. Use the HTTP status endpoint while the service is running, or stop the service first.

### Offline test suite

```bash
python3 -m unittest discover -s tests -v
```

The tests exercise UBX serialization, ACK handling, VALSET/VALGET readback, sign/precision of fixed ECEF coordinates, RTCM CRC and mixed-stream parsing using a simulated receiver. They do **not** constitute end-to-end Docker or GNSS hardware validation.

## Official documentation

- [SparkFun GPS-RTK-SMA breakout (GPS-16481)](https://www.sparkfun.com/sparkfun-gps-rtk-sma-breakout-zed-f9p-qwiic.html)
- [u-blox ZED-F9P Integration Manual](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
- [SparkFun ZED-F9P UBX/NMEA/RTCM interface description](https://cdn.sparkfun.com/assets/learn_tutorials/8/5/6/ZED-F9P_UBX_NMEA_and_RTCM_protocols.pdf)

This project is not affiliated with u-blox or SparkFun.

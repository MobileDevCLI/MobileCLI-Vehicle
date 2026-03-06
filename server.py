#!/usr/bin/env python3
"""
MobileCLI OBD2 Pro Tuner v3.0 — Vehicle Diagnostics & MCU Sensor Dashboard
Runs on localhost:8099, reads /dev/ttyHS1 serial MCU

Architecture (reverse-engineered from EventCenter/canbus2):
  - MCU sends CRLF-delimited binary frames: 0D 0A [LEN] [DATA...LEN bytes]
  - DATA[0] = command type:
      0x71 = System Event (heartbeat, status)
      0x8E = 3D/IMU sensor data (accelerometer/gyro)
      0xA5 = CAN bus relay (vehicle data — requires CAN adapter)
      0xA6 = ATA data (auxiliary)
  - Checksum: ~(sum of LEN + DATA) & 0xFF
  - Send format: 0D 0A [LEN] [DATA] [~CHK] 00

CAN Adapter Status:
  - CAN data requires LuZheng/ZhongHang adapter physically connected to OBD2
  - When connected: 0xA5 frames carry decoded vehicle data (RPM, speed, etc.)
  - Parsed by LuZhengCanParseToyotaFJ: bydata[1]=0x16→Speed, 0x50→RPM, etc.
"""

import http.server
import json
import os
import sys
import time
import threading
import math
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from collections import defaultdict, deque

SERIAL_PORT = "/dev/ttyHS1"
BAUD_RATE = 115200
PORT = 8099

# ── Global State ──
state = {
    "connected": False,
    "can_connected": False,
    "mil": False,
    "dtc_count": 0,  # Updated from heartbeat 0x71 frames when received
    "dtcs": [],
    "last_heartbeat": None,
    "heartbeat_count": 0,
    "raw_frames": [],
    "scan_log": [],
    "frame_counts": {"imu": 0, "heartbeat": 0, "can": 0, "other": 0},
    "frames_per_sec": 0,
    "mcu_raw_bytes": [],
    "vin": "Not available",
    "vehicle": "2008 Toyota FJ Cruiser",
    "protocol": "Szchoiceway MCU Serial",
    "can_status": "No CAN adapter detected",
    "port": SERIAL_PORT,
    "baud": BAUD_RATE,
    "firmware": "GT6-EAU-T16.00.050",
    "mcu_id": "PRLC0__GT6E_GB___S148 7808",
    "head_unit": "Szchoiceway GT6-CAR",
    "platform": "Qualcomm SM6125 / Android 13",
    # IMU sensor data
    "imu": {"x": 0, "y": 0, "z": 0, "b3": 0, "b5": 0, "counter": 0},
    "imu_history": [],
    # CAN vehicle data (populated when CAN adapter connected)
    "can_data": {
        "rpm": None, "speed": None, "throttle": None,
        "coolant_temp": None, "battery_voltage": None,
        "doors": None, "lights": None, "ac": None
    },
    "can_frames": {},
    # Motion detection from IMU
    "motion": {"state": "Unknown", "vibration": 0.0, "tilt_x": 0.0, "tilt_y": 0.0},
    # Data logging
    "log_data": [],
    "log_active": False,
    "log_start": None,
}

# Circular buffers for graphing
imu_graph = deque(maxlen=200)
can_graph = deque(maxlen=200)

# ── MCU Frame Parsing ──
def verify_checksum(frame_bytes):
    """Verify MCU frame checksum: ~(sum of all bytes except last) & 0xFF"""
    if len(frame_bytes) < 3:
        return False
    total = sum(frame_bytes[:-1]) & 0xFF
    expected = (~total) & 0xFF
    return frame_bytes[-1] == expected

def parse_imu_frame(data):
    """Parse 0x8E IMU/3D sensor frame.
    Format: [8E, B2, B3, B4, B5, B6, B7, CHK]
    B2/B4/B6: high nibble = sensor axes (low nibble always 0)
    B3/B5: full byte values (stable baseline values)
    B7: counter/sub-type
    """
    if len(data) < 7:
        return
    b2 = data[1] & 0xFF  # Axis 1 (high nibble)
    b3 = data[2] & 0xFF  # Baseline 1
    b4 = data[3] & 0xFF  # Axis 2 (high nibble)
    b5 = data[4] & 0xFF  # Baseline 2
    b6 = data[5] & 0xFF  # Axis 3 (high nibble)
    b7 = data[6] & 0xFF  # Counter

    # Extract 4-bit axis values from high nibbles
    x = (b2 >> 4) & 0x0F  # 0-15
    y = (b4 >> 4) & 0x0F
    z = (b6 >> 4) & 0x0F

    state["imu"] = {
        "x": x, "y": y, "z": z,
        "b3": b3, "b5": b5,
        "counter": b7,
        "raw": " ".join(f"{b:02X}" for b in data[:8])
    }

    # Motion detection from IMU variance
    now = time.time()
    imu_graph.append({"t": now, "x": x, "y": y, "z": z, "b3": b3, "b5": b5})

    # Calculate vibration (variance of recent readings)
    if len(imu_graph) >= 5:
        recent = list(imu_graph)[-10:]
        x_vals = [r["x"] for r in recent]
        y_vals = [r["y"] for r in recent]
        z_vals = [r["z"] for r in recent]
        variance = sum(
            (sum((v - sum(vals)/len(vals))**2 for v in vals) / len(vals))
            for vals in [x_vals, y_vals, z_vals]
        )
        vibration = math.sqrt(variance)
        if vibration < 0.5:
            motion_state = "Stationary"
        elif vibration < 2.0:
            motion_state = "Idle/Slight Movement"
        elif vibration < 5.0:
            motion_state = "Moving"
        else:
            motion_state = "Heavy Movement"
        state["motion"] = {
            "state": motion_state,
            "vibration": round(vibration, 2),
            "tilt_x": round(x * 6.0 - 48, 1),  # Rough degrees estimate
            "tilt_y": round(y * 6.0 - 48, 1),
        }

    # Data logging
    if state["log_active"]:
        state["log_data"].append({
            "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "x": x, "y": y, "z": z, "b3": b3, "b5": b5, "counter": b7
        })

    state["frame_counts"]["imu"] += 1

def parse_heartbeat_frame(data):
    """Parse 0x71 System Event frame (heartbeat).
    Format varies, but often: [71, STATUS, B3, ...]
    """
    state["last_heartbeat"] = datetime.now().strftime("%H:%M:%S")
    state["heartbeat_count"] += 1
    if len(data) >= 2:
        status = data[1] & 0xFF
        state["mil"] = bool(status & 0x80)
        state["dtc_count"] = status & 0x7F
    state["frame_counts"]["heartbeat"] += 1

def parse_can_frame(data):
    """Parse 0xA5 CAN bus relay frame.
    This is the raw CAN data from the LuZheng adapter.
    It will be further parsed by the canbus2 frame header format (default: 2E2E).
    """
    state["can_connected"] = True
    state["can_status"] = "CAN adapter connected"
    state["frame_counts"]["can"] += 1

    # Store raw CAN frame
    hex_str = " ".join(f"{b:02X}" for b in data)
    state["can_frames"][time.time()] = hex_str

    # Try to decode LuZheng 2E2E format
    # Header: 2E 2E LEN [bydata...] CHK
    # We'll look for the inner command format
    decode_luzheng_can(data)

def decode_luzheng_can(data):
    """Decode LuZheng CAN protocol for Toyota FJ Cruiser.
    processCmd routing: bydata[1] = command ID
    """
    # The CAN data might be wrapped in 2E2E or direct format
    # Try to find processCmd-compatible data
    if len(data) < 4:
        return

    # Try direct format (bydata starts at data[0] or data[1])
    for offset in [0, 1]:
        if offset + 1 >= len(data):
            continue
        cmd_id = data[offset + 1] if offset + 1 < len(data) else 0

        if cmd_id == 0x16:  # Speed
            if offset + 4 < len(data):
                speed_raw = (data[offset+3] & 0xFF) + (data[offset+4] & 0xFF) * 256
                speed = speed_raw / 16.0
                state["can_data"]["speed"] = round(speed, 1)
                can_graph.append({"t": time.time(), "speed": speed})

        elif cmd_id == 0x50:  # RPM
            if offset + 4 < len(data):
                rpm = (data[offset+4] & 0xFF) * 256 + (data[offset+3] & 0xFF)
                state["can_data"]["rpm"] = rpm
                can_graph.append({"t": time.time(), "rpm": rpm})

        elif cmd_id == 0x24:  # Doors/Lights
            state["can_data"]["doors"] = f"0x{data[offset+2]:02X}" if offset+2 < len(data) else None

        elif cmd_id == 0x28:  # AC
            state["can_data"]["ac"] = f"0x{data[offset+2]:02X}" if offset+2 < len(data) else None

def parse_frames(raw_data):
    """Parse CRLF-delimited MCU frames from raw serial data."""
    frames = raw_data.split(b'\r\n')
    parsed = 0
    for frame in frames:
        if len(frame) < 2:
            continue
        length = frame[0]
        data = frame[1:]  # Command type + payload + checksum

        if len(data) < length:
            continue  # Incomplete frame

        cmd_type = data[0] & 0xFF

        # Store raw frame for display
        hex_str = " ".join(f"{b:02X}" for b in frame)
        state["raw_frames"].append({
            "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "hex": hex_str,
            "type": {0x71: "SysEvent", 0x8E: "IMU/3D", 0xA5: "CAN", 0xA6: "ATA"}.get(cmd_type, f"0x{cmd_type:02X}"),
            "len": length
        })
        if len(state["raw_frames"]) > 500:
            state["raw_frames"] = state["raw_frames"][-200:]

        # Dispatch by command type
        if cmd_type == 0x8E:
            parse_imu_frame(data)
        elif cmd_type == 0x71:
            parse_heartbeat_frame(data)
        elif cmd_type == 0xA5:
            parse_can_frame(data)
        else:
            state["frame_counts"]["other"] += 1

        parsed += 1
    return parsed

# ── Serial Reader Thread ──
serial_fd = None

def open_serial():
    global serial_fd
    try:
        serial_fd = os.open(SERIAL_PORT, os.O_RDWR | os.O_NONBLOCK)
        state["connected"] = True
        add_log("Serial port opened: " + SERIAL_PORT)
        return True
    except Exception as e:
        state["connected"] = False
        add_log(f"Serial open failed: {e}")
        return False

def background_reader():
    """Background thread reading serial data continuously."""
    global serial_fd
    fps_counter = 0
    fps_time = time.time()

    while True:
        if serial_fd is None:
            if not open_serial():
                time.sleep(2)
                continue

        try:
            data = os.read(serial_fd, 4096)
            if data:
                count = parse_frames(data)
                fps_counter += count

                # Update FPS
                now = time.time()
                if now - fps_time >= 1.0:
                    state["frames_per_sec"] = fps_counter
                    fps_counter = 0
                    fps_time = now
        except BlockingIOError:
            time.sleep(0.05)
        except OSError:
            state["connected"] = False
            serial_fd = None
            time.sleep(1)

def add_log(msg):
    entry = {"time": datetime.now().strftime("%H:%M:%S"), "msg": msg}
    state["scan_log"].append(entry)
    if len(state["scan_log"]) > 200:
        state["scan_log"] = state["scan_log"][-100:]

# ── Web Server ──
class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress access logs

    def do_GET(self):
        path = urlparse(self.path).path
        params = parse_qs(urlparse(self.path).query)

        if path == "/api/state":
            self.json_response(get_api_state())
        elif path == "/api/imu_graph":
            self.json_response(list(imu_graph))
        elif path == "/api/can_graph":
            self.json_response(list(can_graph))
        elif path == "/api/raw":
            self.json_response(state["raw_frames"][-100:])
        elif path == "/api/log":
            self.json_response(state["scan_log"])
        elif path == "/api/log_data":
            self.json_response(state["log_data"][-1000:])
        elif path == "/api/start_log":
            state["log_active"] = True
            state["log_start"] = datetime.now().strftime("%Y%m%d_%H%M%S")
            state["log_data"] = []
            self.json_response({"status": "logging started"})
        elif path == "/api/stop_log":
            state["log_active"] = False
            self.json_response({"status": "logging stopped", "records": len(state["log_data"])})
        elif path == "/api/download_log":
            self.send_csv_log()
        else:
            self.send_dashboard()

    def json_response(self, data):
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(body))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def send_csv_log(self):
        lines = ["time,x,y,z,b3,b5,counter"]
        for r in state["log_data"]:
            lines.append(f"{r['time']},{r['x']},{r['y']},{r['z']},{r['b3']},{r['b5']},{r['counter']}")
        body = "\n".join(lines).encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/csv")
        self.send_header("Content-Disposition", f"attachment; filename=imu_log_{state.get('log_start','data')}.csv")
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def send_dashboard(self):
        body = DASHBOARD_HTML.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

def get_api_state():
    return {
        "connected": state["connected"],
        "can_connected": state["can_connected"],
        "can_status": state["can_status"],
        "mil": state["mil"],
        "dtc_count": state["dtc_count"],
        "heartbeat": state["last_heartbeat"],
        "heartbeat_count": state["heartbeat_count"],
        "fps": state["frames_per_sec"],
        "frame_counts": state["frame_counts"],
        "imu": state["imu"],
        "motion": state["motion"],
        "can_data": state["can_data"],
        "vehicle": state["vehicle"],
        "protocol": state["protocol"],
        "port": state["port"],
        "baud": state["baud"],
        "firmware": state["firmware"],
        "mcu_id": state["mcu_id"],
        "head_unit": state["head_unit"],
        "platform": state["platform"],
        "log_active": state["log_active"],
        "log_records": len(state["log_data"]),
    }

# ── Dashboard HTML ──
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>OBD2 Pro Tuner v3.0</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body { font-family: 'Segoe UI',system-ui,sans-serif; background:#0a0e17; color:#e0e6ed; }
.header { background:linear-gradient(135deg,#1a1f2e,#0d1117); padding:12px 20px;
  display:flex; justify-content:space-between; align-items:center; border-bottom:1px solid #21262d; }
.header h1 { font-size:18px; color:#58a6ff; }
.header .status { display:flex; gap:12px; font-size:12px; }
.indicator { display:inline-flex; align-items:center; gap:4px; }
.dot { width:8px; height:8px; border-radius:50%; }
.dot.green { background:#3fb950; box-shadow:0 0 6px #3fb95088; }
.dot.red { background:#f85149; box-shadow:0 0 6px #f8514988; }
.dot.yellow { background:#d29922; box-shadow:0 0 6px #d2992288; }
.dot.blue { background:#58a6ff; box-shadow:0 0 6px #58a6ff88; }
.tabs { display:flex; background:#161b22; border-bottom:1px solid #21262d; overflow-x:auto; }
.tab { padding:10px 16px; cursor:pointer; font-size:13px; white-space:nowrap;
  color:#8b949e; border-bottom:2px solid transparent; transition:all .2s; }
.tab:hover { color:#c9d1d9; }
.tab.active { color:#58a6ff; border-bottom-color:#58a6ff; }
.content { padding:16px; min-height:calc(100vh - 100px); }
.panel { display:none; }
.panel.active { display:block; }
.grid { display:grid; gap:12px; }
.grid-2 { grid-template-columns:1fr 1fr; }
.grid-3 { grid-template-columns:1fr 1fr 1fr; }
.grid-4 { grid-template-columns:1fr 1fr 1fr 1fr; }
@media(max-width:768px) { .grid-2,.grid-3,.grid-4 { grid-template-columns:1fr; } }
.card { background:#161b22; border:1px solid #21262d; border-radius:8px; padding:14px; }
.card h3 { font-size:12px; color:#8b949e; text-transform:uppercase; letter-spacing:1px; margin-bottom:8px; }
.card .value { font-size:28px; font-weight:700; color:#e6edf3; }
.card .value.small { font-size:18px; }
.card .unit { font-size:12px; color:#8b949e; margin-left:4px; }
.card .sub { font-size:11px; color:#6e7681; margin-top:4px; }
.card.highlight { border-color:#58a6ff44; }
.card.warn { border-color:#d2992244; }
.card.danger { border-color:#f8514944; }
.card.can-active { border-color:#3fb95044; background:#0d1f0d; }
.badge { display:inline-block; padding:2px 8px; border-radius:10px; font-size:11px; font-weight:600; }
.badge.ok { background:#0d3117; color:#3fb950; }
.badge.warn { background:#3d2b00; color:#d29922; }
.badge.err { background:#3d0b0b; color:#f85149; }
.badge.info { background:#0d2744; color:#58a6ff; }
.raw-box { background:#0d1117; border:1px solid #21262d; border-radius:6px;
  padding:10px; font-family:'Cascadia Code',monospace; font-size:12px;
  max-height:400px; overflow-y:auto; line-height:1.6; }
.raw-line { color:#7ee787; }
.raw-line .ts { color:#6e7681; }
.raw-line .type { color:#d2a8ff; font-weight:600; }
canvas { width:100%; height:200px; background:#0d1117; border:1px solid #21262d; border-radius:6px; }
table { width:100%; border-collapse:collapse; font-size:13px; }
th { text-align:left; padding:8px; color:#8b949e; border-bottom:1px solid #21262d; font-weight:600; }
td { padding:6px 8px; border-bottom:1px solid #21262d11; }
.btn { padding:8px 16px; border:1px solid #30363d; border-radius:6px;
  background:#21262d; color:#c9d1d9; cursor:pointer; font-size:13px; }
.btn:hover { background:#30363d; }
.btn.primary { background:#238636; border-color:#2ea043; color:white; }
.btn.danger { background:#da3633; border-color:#f85149; color:white; }
.gauge { position:relative; width:120px; height:120px; margin:0 auto; }
.motion-viz { text-align:center; padding:20px; }
.motion-state { font-size:24px; font-weight:700; margin:10px 0; }
.info-grid { display:grid; grid-template-columns:140px 1fr; gap:4px 12px; font-size:13px; }
.info-grid .label { color:#8b949e; }
.info-grid .val { color:#e6edf3; font-family:monospace; }
</style>
</head>
<body>

<div class="header">
  <h1>OBD2 Pro Tuner v3.0</h1>
  <div class="status">
    <span class="indicator"><span class="dot" id="dot-serial"></span> <span id="lbl-serial">MCU</span></span>
    <span class="indicator"><span class="dot" id="dot-can"></span> <span id="lbl-can">CAN</span></span>
    <span id="fps-display" style="color:#6e7681"></span>
  </div>
</div>

<div class="tabs" id="tabs">
  <div class="tab active" data-tab="dashboard">Dashboard</div>
  <div class="tab" data-tab="sensors">Sensors</div>
  <div class="tab" data-tab="vehicle">Vehicle Data</div>
  <div class="tab" data-tab="raw">Raw Hex</div>
  <div class="tab" data-tab="graphs">Graphs</div>
  <div class="tab" data-tab="log">Data Log</div>
  <div class="tab" data-tab="tools">Tools</div>
  <div class="tab" data-tab="info">System Info</div>
</div>

<div class="content">

<!-- Dashboard -->
<div class="panel active" id="panel-dashboard">
  <div class="grid grid-4" style="margin-bottom:12px">
    <div class="card" id="card-motion">
      <h3>Motion State</h3>
      <div class="value small" id="val-motion">--</div>
      <div class="sub" id="val-vibration">Vibration: --</div>
    </div>
    <div class="card" id="card-imu-x">
      <h3>Accel X</h3>
      <div class="value" id="val-imu-x">--</div>
      <div class="sub">Lateral axis</div>
    </div>
    <div class="card" id="card-imu-y">
      <h3>Accel Y</h3>
      <div class="value" id="val-imu-y">--</div>
      <div class="sub">Longitudinal axis</div>
    </div>
    <div class="card" id="card-imu-z">
      <h3>Accel Z</h3>
      <div class="value" id="val-imu-z">--</div>
      <div class="sub">Vertical axis</div>
    </div>
  </div>

  <div class="grid grid-2">
    <div class="card">
      <h3>MCU Connection</h3>
      <div id="dash-connection" style="font-size:13px; line-height:1.8">
        Loading...
      </div>
    </div>
    <div class="card" id="card-can-status">
      <h3>CAN Adapter Status</h3>
      <div id="dash-can" style="font-size:13px; line-height:1.8">
        Loading...
      </div>
    </div>
  </div>

  <div class="card" style="margin-top:12px" id="card-vehicle-data">
    <h3>Vehicle Data (CAN)</h3>
    <div id="dash-vehicle-data">
      <div style="color:#8b949e; padding:10px;">
        No CAN adapter detected. Vehicle data (RPM, speed, throttle) requires a
        CAN bus adapter connected to the OBD2 port. The head unit's LuZheng adapter
        must be wired to the vehicle harness.
        <br><br>
        <b>Current data source:</b> MCU 3D/IMU sensor (accelerometer/gyroscope)
      </div>
    </div>
  </div>
</div>

<!-- Sensors -->
<div class="panel" id="panel-sensors">
  <div class="card" style="margin-bottom:12px">
    <h3>IMU / 3D Sensor Data (0x8E Frames)</h3>
    <div class="grid grid-3" style="margin-top:10px">
      <div>
        <div style="color:#8b949e; font-size:12px">Axis X (High Nibble B2)</div>
        <div style="font-size:32px; font-weight:700; color:#f0883e" id="sens-x">--</div>
      </div>
      <div>
        <div style="color:#8b949e; font-size:12px">Axis Y (High Nibble B4)</div>
        <div style="font-size:32px; font-weight:700; color:#3fb950" id="sens-y">--</div>
      </div>
      <div>
        <div style="color:#8b949e; font-size:12px">Axis Z (High Nibble B6)</div>
        <div style="font-size:32px; font-weight:700; color:#58a6ff" id="sens-z">--</div>
      </div>
    </div>
  </div>

  <div class="grid grid-2">
    <div class="card">
      <h3>Baseline Values</h3>
      <table>
        <tr><th>Parameter</th><th>Value</th><th>Hex</th></tr>
        <tr><td>B3 (Baseline 1)</td><td id="sens-b3">--</td><td id="sens-b3-hex">--</td></tr>
        <tr><td>B5 (Baseline 2)</td><td id="sens-b5">--</td><td id="sens-b5-hex">--</td></tr>
        <tr><td>Counter (B7)</td><td id="sens-ctr">--</td><td id="sens-ctr-hex">--</td></tr>
      </table>
    </div>
    <div class="card">
      <h3>Motion Analysis</h3>
      <div class="motion-viz">
        <div class="motion-state" id="motion-state">--</div>
        <div style="color:#8b949e; font-size:13px">
          Vibration: <span id="motion-vib">--</span><br>
          Tilt X: <span id="motion-tx">--</span>&deg; &nbsp;
          Tilt Y: <span id="motion-ty">--</span>&deg;
        </div>
      </div>
    </div>
  </div>

  <div class="card" style="margin-top:12px">
    <h3>Raw Frame (Last)</h3>
    <div id="sens-raw" style="font-family:monospace; font-size:14px; color:#7ee787; padding:8px">--</div>
  </div>
</div>

<!-- Vehicle Data -->
<div class="panel" id="panel-vehicle">
  <div id="vehicle-can-active" style="display:none">
    <div class="grid grid-3" style="margin-bottom:12px">
      <div class="card can-active">
        <h3>RPM</h3>
        <div class="value" id="veh-rpm">--</div>
      </div>
      <div class="card can-active">
        <h3>Speed</h3>
        <div class="value" id="veh-speed">--<span class="unit">km/h</span></div>
      </div>
      <div class="card can-active">
        <h3>Throttle</h3>
        <div class="value" id="veh-throttle">--<span class="unit">%</span></div>
      </div>
    </div>
    <div class="grid grid-2">
      <div class="card can-active">
        <h3>Doors / Lights</h3>
        <div id="veh-doors" style="font-size:14px">--</div>
      </div>
      <div class="card can-active">
        <h3>AC / Climate</h3>
        <div id="veh-ac" style="font-size:14px">--</div>
      </div>
    </div>
  </div>
  <div id="vehicle-no-can" class="card">
    <h3>CAN Adapter Required</h3>
    <div style="padding:20px; color:#8b949e; line-height:1.8">
      <p style="font-size:16px; color:#d29922; margin-bottom:12px">
        No CAN bus adapter detected on MCU serial port
      </p>
      <p>To get live vehicle data (RPM, Speed, Throttle, Temperature, Doors, AC), the
      head unit needs a CAN bus adapter module connected:</p>
      <ul style="margin:12px 0 12px 20px">
        <li><b>LuZheng CAN Adapter</b> — Connects to vehicle OBD2 harness</li>
        <li><b>ZhongHang TY Adapter</b> — Alternative CAN protocol</li>
        <li><b>Wiring:</b> OBD2 connector &#8594; CAN adapter &#8594; Head unit MCU</li>
      </ul>
      <p>When a CAN adapter is connected, this page will automatically display decoded
      vehicle parameters using the <b>LuZhengCanParseToyotaFJ</b> protocol parser
      (Car Type 0x22E = Toyota FJ Cruiser).</p>
      <hr style="border-color:#21262d; margin:16px 0">
      <p style="font-size:12px; color:#6e7681">
        <b>Technical:</b> MCU command 0xA5 = CAN relay data. Currently only receiving
        0x8E (IMU/3D sensor). The MCU's CAN transceiver is not receiving vehicle CAN frames.
        Checked: Sys_Vehicle_deries=3 (LuZheng), Sys_CarType=558 (0x22E, FJ Cruiser).
      </p>
    </div>
  </div>
</div>

<!-- Raw Hex -->
<div class="panel" id="panel-raw">
  <div class="card">
    <h3>Live Frame Stream</h3>
    <div style="margin-bottom:8px">
      <span class="badge info" id="raw-count">0 frames</span>
      <span class="badge" id="raw-imu" style="background:#2d1f00;color:#f0883e">IMU: 0</span>
      <span class="badge" id="raw-hb" style="background:#0d2744;color:#58a6ff">Heartbeat: 0</span>
      <span class="badge" id="raw-can" style="background:#0d3117;color:#3fb950">CAN: 0</span>
    </div>
    <div class="raw-box" id="raw-box"></div>
  </div>
</div>

<!-- Graphs -->
<div class="panel" id="panel-graphs">
  <div class="card" style="margin-bottom:12px">
    <h3>IMU Sensor Waveform</h3>
    <canvas id="imu-canvas" height="200"></canvas>
  </div>
  <div class="card">
    <h3>Baseline Values (B3/B5)</h3>
    <canvas id="baseline-canvas" height="200"></canvas>
  </div>
</div>

<!-- Data Log -->
<div class="panel" id="panel-log">
  <div class="card">
    <h3>Data Logger</h3>
    <div style="margin-bottom:12px; display:flex; gap:8px; align-items:center">
      <button class="btn primary" id="btn-log-start" onclick="startLog()">Start Recording</button>
      <button class="btn danger" id="btn-log-stop" onclick="stopLog()" style="display:none">Stop Recording</button>
      <button class="btn" onclick="downloadLog()">Download CSV</button>
      <span id="log-status" style="color:#8b949e; font-size:13px"></span>
    </div>
    <div class="raw-box" id="log-box" style="max-height:300px">No data recorded yet.</div>
  </div>
</div>

<!-- Tools -->
<div class="panel" id="panel-tools">
  <div class="grid grid-2">
    <div class="card">
      <h3>MCU Protocol Info</h3>
      <div style="font-size:13px; color:#8b949e; line-height:1.8">
        <b>Frame Format:</b> 0D 0A [LEN] [CMD DATA...] [~CHK] 00<br>
        <b>Read Format:</b> 0D 0A [LEN] [CMD DATA...CHK]<br>
        <b>Command Types:</b><br>
        &nbsp; 0x71 = System Event (heartbeat)<br>
        &nbsp; 0x8E = 3D/IMU sensor data<br>
        &nbsp; 0xA5 = CAN bus relay (vehicle data)<br>
        &nbsp; 0xA6 = ATA auxiliary data<br>
        <b>CAN Decode:</b> LuZheng 2E2E header format<br>
        <b>Car Type:</b> 0x22E = Toyota FJ Cruiser
      </div>
    </div>
    <div class="card">
      <h3>Diagnostic Commands</h3>
      <div style="font-size:13px; color:#8b949e; line-height:1.8">
        <p>Available when CAN adapter is connected:</p>
        <table>
          <tr><td style="color:#d2a8ff">0x16</td><td>Vehicle Speed</td></tr>
          <tr><td style="color:#d2a8ff">0x50</td><td>Engine RPM</td></tr>
          <tr><td style="color:#d2a8ff">0x24</td><td>Doors / Lights</td></tr>
          <tr><td style="color:#d2a8ff">0x28</td><td>AC / Climate</td></tr>
          <tr><td style="color:#d2a8ff">0x1D</td><td>Radar (Front)</td></tr>
          <tr><td style="color:#d2a8ff">0x1E</td><td>Radar (Rear)</td></tr>
          <tr><td style="color:#d2a8ff">0x29</td><td>Wheel Status</td></tr>
          <tr><td style="color:#d2a8ff">0x30-32</td><td>CAN Version</td></tr>
        </table>
      </div>
    </div>
  </div>
</div>

<!-- System Info -->
<div class="panel" id="panel-info">
  <div class="grid grid-2">
    <div class="card">
      <h3>Vehicle</h3>
      <div class="info-grid" id="info-vehicle"></div>
    </div>
    <div class="card">
      <h3>Head Unit</h3>
      <div class="info-grid" id="info-hu"></div>
    </div>
  </div>
  <div class="card" style="margin-top:12px">
    <h3>Connection Status</h3>
    <div class="info-grid" id="info-conn"></div>
  </div>
</div>

</div>

<script>
let activeTab = 'dashboard';

// Tab switching
document.querySelectorAll('.tab').forEach(tab => {
  tab.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
    tab.classList.add('active');
    activeTab = tab.dataset.tab;
    document.getElementById('panel-' + activeTab).classList.add('active');
  });
});

// API polling
async function poll() {
  try {
    const res = await fetch('/api/state');
    const d = await res.json();
    updateDashboard(d);
    if (activeTab === 'raw') updateRaw();
    if (activeTab === 'graphs') updateGraphs();
    if (activeTab === 'log') updateLogPanel();
  } catch(e) {}
  setTimeout(poll, 500);
}

function updateDashboard(d) {
  // Status indicators
  const dotSerial = document.getElementById('dot-serial');
  const dotCan = document.getElementById('dot-can');
  dotSerial.className = 'dot ' + (d.connected ? 'green' : 'red');
  dotCan.className = 'dot ' + (d.can_connected ? 'green' : 'yellow');
  document.getElementById('lbl-serial').textContent = d.connected ? 'MCU Connected' : 'MCU Disconnected';
  document.getElementById('lbl-can').textContent = d.can_connected ? 'CAN Active' : 'No CAN';
  document.getElementById('fps-display').textContent = d.fps + ' fps';

  // Dashboard cards
  const imu = d.imu || {};
  document.getElementById('val-imu-x').textContent = imu.x !== undefined ? imu.x : '--';
  document.getElementById('val-imu-y').textContent = imu.y !== undefined ? imu.y : '--';
  document.getElementById('val-imu-z').textContent = imu.z !== undefined ? imu.z : '--';

  const motion = d.motion || {};
  document.getElementById('val-motion').textContent = motion.state || '--';
  document.getElementById('val-vibration').textContent = 'Vibration: ' + (motion.vibration || 0);

  const cardMotion = document.getElementById('card-motion');
  if (motion.state === 'Stationary') cardMotion.className = 'card highlight';
  else if (motion.state === 'Moving' || motion.state === 'Heavy Movement') cardMotion.className = 'card warn';
  else cardMotion.className = 'card';

  // Connection info
  document.getElementById('dash-connection').innerHTML = `
    <span class="indicator"><span class="dot ${d.connected?'green':'red'}"></span> Serial: ${d.port}</span><br>
    Baud: ${d.baud} | Frames: ${d.frame_counts.imu + d.frame_counts.heartbeat + d.frame_counts.can + d.frame_counts.other}<br>
    IMU: ${d.frame_counts.imu} | Heartbeat: ${d.frame_counts.heartbeat} | CAN: ${d.frame_counts.can}<br>
    Last Heartbeat: ${d.heartbeat || 'None yet'} (${d.heartbeat_count} total)
    ${d.mil ? '<br><span class="badge err">MIL ON</span>' : ''}
    ${d.dtc_count > 0 ? '<br>DTCs: ' + d.dtc_count : ''}
  `;

  document.getElementById('dash-can').innerHTML = `
    <span class="badge ${d.can_connected ? 'ok' : 'warn'}">${d.can_status}</span><br><br>
    ${d.can_connected ? 'Receiving CAN frames from vehicle' :
      'MCU serial shows only IMU/3D data (0x8E frames).<br>' +
      'No CAN relay frames (0xA5) detected.<br><br>' +
      'The LuZheng CAN adapter may not be wired to the vehicle OBD2 port.'}
  `;

  // Vehicle data panel
  if (d.can_connected) {
    document.getElementById('vehicle-can-active').style.display = 'block';
    document.getElementById('vehicle-no-can').style.display = 'none';
    const cd = d.can_data;
    document.getElementById('veh-rpm').textContent = cd.rpm !== null ? cd.rpm : '--';
    document.getElementById('veh-speed').innerHTML = (cd.speed !== null ? cd.speed : '--') + '<span class="unit">km/h</span>';
    document.getElementById('veh-doors').textContent = cd.doors || '--';
    document.getElementById('veh-ac').textContent = cd.ac || '--';
  } else {
    document.getElementById('vehicle-can-active').style.display = 'none';
    document.getElementById('vehicle-no-can').style.display = 'block';
  }

  // Sensor panel
  if (activeTab === 'sensors') {
    document.getElementById('sens-x').textContent = imu.x !== undefined ? imu.x : '--';
    document.getElementById('sens-y').textContent = imu.y !== undefined ? imu.y : '--';
    document.getElementById('sens-z').textContent = imu.z !== undefined ? imu.z : '--';
    document.getElementById('sens-b3').textContent = imu.b3 || '--';
    document.getElementById('sens-b3-hex').textContent = imu.b3 !== undefined ? '0x' + imu.b3.toString(16).toUpperCase().padStart(2,'0') : '--';
    document.getElementById('sens-b5').textContent = imu.b5 || '--';
    document.getElementById('sens-b5-hex').textContent = imu.b5 !== undefined ? '0x' + imu.b5.toString(16).toUpperCase().padStart(2,'0') : '--';
    document.getElementById('sens-ctr').textContent = imu.counter !== undefined ? imu.counter : '--';
    document.getElementById('sens-ctr-hex').textContent = imu.counter !== undefined ? '0x' + imu.counter.toString(16).toUpperCase().padStart(2,'0') : '--';
    document.getElementById('sens-raw').textContent = imu.raw || '--';
    document.getElementById('motion-state').textContent = motion.state || '--';
    document.getElementById('motion-vib').textContent = motion.vibration || '0';
    document.getElementById('motion-tx').textContent = motion.tilt_x || '0';
    document.getElementById('motion-ty').textContent = motion.tilt_y || '0';
  }

  // System info
  if (activeTab === 'info') {
    document.getElementById('info-vehicle').innerHTML = infoRow('Vehicle', d.vehicle) +
      infoRow('VIN', 'Not available') +
      infoRow('CAN Profile', d.can_connected ? 'LuZheng Toyota FJ' : 'Not active') +
      infoRow('Car Type ID', '0x22E (558)');
    document.getElementById('info-hu').innerHTML = infoRow('Model', d.head_unit) +
      infoRow('Firmware', d.firmware) +
      infoRow('MCU ID', d.mcu_id) +
      infoRow('Platform', d.platform);
    document.getElementById('info-conn').innerHTML = infoRow('Serial Port', d.port) +
      infoRow('Baud Rate', d.baud) +
      infoRow('Protocol', d.protocol) +
      infoRow('CAN Status', d.can_status) +
      infoRow('Data Source', d.can_connected ? 'CAN Adapter (0xA5)' : 'IMU Sensor (0x8E)') +
      infoRow('Frame Rate', d.fps + ' fps');
  }

  // Frame count badges
  document.getElementById('raw-imu').textContent = 'IMU: ' + d.frame_counts.imu;
  document.getElementById('raw-hb').textContent = 'HB: ' + d.frame_counts.heartbeat;
  document.getElementById('raw-can').textContent = 'CAN: ' + d.frame_counts.can;
}

function infoRow(label, value) {
  return `<div class="label">${label}</div><div class="val">${value}</div>`;
}

async function updateRaw() {
  try {
    const res = await fetch('/api/raw');
    const frames = await res.json();
    const box = document.getElementById('raw-box');
    let html = '';
    for (const f of frames.slice(-50)) {
      const typeColor = f.type === 'IMU/3D' ? '#f0883e' : f.type === 'CAN' ? '#3fb950' : '#58a6ff';
      html += `<div class="raw-line"><span class="ts">${f.time}</span> <span class="type" style="color:${typeColor}">[${f.type}]</span> ${f.hex}</div>`;
    }
    box.innerHTML = html;
    box.scrollTop = box.scrollHeight;
    document.getElementById('raw-count').textContent = frames.length + ' frames';
  } catch(e) {}
}

async function updateGraphs() {
  try {
    const res = await fetch('/api/imu_graph');
    const data = await res.json();
    drawGraph('imu-canvas', data, ['x','y','z'], ['#f0883e','#3fb950','#58a6ff'], 0, 15);
    drawGraph('baseline-canvas', data, ['b3','b5'], ['#d2a8ff','#d29922'], 0, 255);
  } catch(e) {}
}

function drawGraph(canvasId, data, keys, colors, ymin, ymax) {
  const canvas = document.getElementById(canvasId);
  if (!canvas || !data.length) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width = canvas.offsetWidth * 2;
  const h = canvas.height = 400;
  ctx.clearRect(0, 0, w, h);

  // Grid
  ctx.strokeStyle = '#21262d';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = (h * i) / 4;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    ctx.fillStyle = '#6e7681';
    ctx.font = '20px monospace';
    ctx.fillText(Math.round(ymax - (ymax-ymin)*i/4), 4, y + 16);
  }

  // Data lines
  const n = data.length;
  keys.forEach((key, ki) => {
    ctx.strokeStyle = colors[ki];
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < n; i++) {
      const x = (i / (n - 1)) * w;
      const val = data[i][key] || 0;
      const y = h - ((val - ymin) / (ymax - ymin)) * h;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  });

  // Legend
  keys.forEach((key, ki) => {
    ctx.fillStyle = colors[ki];
    ctx.font = 'bold 22px sans-serif';
    ctx.fillText(key.toUpperCase(), w - 200 + ki * 60, 30);
  });
}

async function updateLogPanel() {
  const st = document.getElementById('log-status');
  try {
    const res = await fetch('/api/state');
    const d = await res.json();
    if (d.log_active) {
      st.textContent = 'Recording... ' + d.log_records + ' records';
      document.getElementById('btn-log-start').style.display = 'none';
      document.getElementById('btn-log-stop').style.display = 'inline-block';
    } else {
      st.textContent = d.log_records > 0 ? d.log_records + ' records captured' : '';
      document.getElementById('btn-log-start').style.display = 'inline-block';
      document.getElementById('btn-log-stop').style.display = 'none';
    }
  } catch(e) {}
}

async function startLog() { await fetch('/api/start_log'); }
async function stopLog() { await fetch('/api/stop_log'); }
function downloadLog() { window.open('/api/download_log'); }

poll();
</script>
</body>
</html>"""

# ── Main ──
if __name__ == "__main__":
    add_log("OBD2 Pro Tuner v3.0 starting...")
    add_log(f"Serial: {SERIAL_PORT} @ {BAUD_RATE}")
    add_log("Protocol: CRLF-delimited MCU frames")
    add_log("Monitoring for: 0x8E (IMU), 0x71 (Heartbeat), 0xA5 (CAN)")

    reader = threading.Thread(target=background_reader, daemon=True)
    reader.start()

    server = HTTPServer(("0.0.0.0", PORT), Handler)
    add_log(f"Dashboard: http://localhost:{PORT}")
    print(f"OBD2 Pro Tuner v3.0 running on port {PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()

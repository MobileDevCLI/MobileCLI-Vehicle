#!/usr/bin/env python3
"""
MobileCLI OBD2 Pro Tuner — Web-based vehicle diagnostics & tuning interface
Runs on localhost:8099, communicates with /dev/ttyHS1 serial MCU

Architecture:
  - MCU sends CRLF-delimited binary frames on /dev/ttyHS1 @ 115200 baud
  - Frame types: 0x03(status), 0x04(heartbeat), 0x08(CAN relay), 0x11(extended)
  - Heartbeat 0x04: MIL status and DTC count (ACCURATE)
  - CAN relay 0x08: raw CAN bus data from vehicle (requires CAN service decode)
  - Checksum: ~(sum of all bytes except last) & 0xFF
"""

import http.server
import json
import os
import sys
import time
import threading
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from collections import defaultdict

SERIAL_PORT = "/dev/ttyHS1"
BAUD_RATE = 115200
PORT = 8099

# ── Global State ──
state = {
    "connected": False,
    "mil": False,
    "dtc_count": 1,  # Last known from raw capture: 04 71 01 00 89
    "dtcs": [],
    "last_heartbeat": None,
    "heartbeat_count": 0,
    "raw_frames": [],
    "scan_log": [],
    "frame_counts": {"sensor": 0, "heartbeat": 0, "status": 0, "other": 0},
    "frames_per_sec": 0,
    "mcu_raw_bytes": [],
    "vin": "Not available",
    "vehicle": "2008 Toyota FJ Cruiser",
    "protocol": "Szchoiceway LuZheng CAN",
    "canbus_profile": "CARTYPE_TOYOTA_FJ_CRUISER",
    "port": SERIAL_PORT,
    "baud": BAUD_RATE,
    "firmware": "GT6-EAU-T16.00.050",
    "mcu_id": "PRLC0__GT6E_GB___S148 7808",
    "head_unit": "Szchoiceway GT6-CAR",
    "platform": "Qualcomm SM6125 / Android 13",
    "can_groups": {},
    "can_signals": {},  # Tracked CAN signal values per B7 group
    "can_history": [],
    "data_log": [],     # CSV-style data log entries
    "logging_active": False,
    "start_time": time.time(),
}

serial_fd = None
serial_lock = threading.Lock()
running = True

# ── DTC Database ──
DTC_DB = {
    "P0010": ("Intake Camshaft Position Actuator Circuit (Bank 1)", "Check wiring to camshaft position actuator.", "Medium"),
    "P0011": ("Intake Cam Position Timing Over-Advanced (Bank 1)", "Check engine oil level/condition. Replace camshaft position actuator solenoid.", "High"),
    "P0100": ("MAF Circuit Malfunction", "Clean MAF sensor. Check wiring. Replace MAF if cleaning fails.", "High"),
    "P0101": ("MAF Circuit Range/Performance", "Clean MAF sensor with MAF cleaner spray. Check for air leaks.", "Medium"),
    "P0120": ("Throttle Position Sensor Circuit", "Check TPS wiring. Test TPS voltage sweep. Replace TPS.", "High"),
    "P0128": ("Coolant Thermostat Below Regulating Temp", "Replace thermostat. Check ECT sensor accuracy.", "Medium"),
    "P0130": ("O2 Sensor Circuit (Bank 1 Sensor 1)", "Test O2 sensor voltage. Check wiring. Replace O2 sensor.", "Medium"),
    "P0171": ("System Too Lean (Bank 1)", "Check for vacuum/intake leaks. Clean MAF. Check fuel pressure.", "High"),
    "P0172": ("System Too Rich (Bank 1)", "Check for leaking injectors. Inspect fuel pressure regulator.", "High"),
    "P0174": ("System Too Lean (Bank 2)", "Check for vacuum leaks on bank 2. Clean MAF.", "High"),
    "P0175": ("System Too Rich (Bank 2)", "Check injectors bank 2. Inspect fuel pressure.", "High"),
    "P0300": ("Random/Multiple Cylinder Misfire", "Check spark plugs, coils, fuel injectors. Check compression.", "Critical"),
    "P0301": ("Cylinder 1 Misfire", "Replace spark plug #1. Swap coil to test. Check injector #1.", "High"),
    "P0302": ("Cylinder 2 Misfire", "Replace spark plug #2. Swap coil to test.", "High"),
    "P0303": ("Cylinder 3 Misfire", "Replace spark plug #3. Swap coil to test.", "High"),
    "P0304": ("Cylinder 4 Misfire", "Replace spark plug #4. Swap coil to test.", "High"),
    "P0305": ("Cylinder 5 Misfire", "Replace spark plug #5. Swap coil to test.", "High"),
    "P0306": ("Cylinder 6 Misfire", "Replace spark plug #6. Swap coil to test.", "High"),
    "P0325": ("Knock Sensor 1 Circuit (Bank 1)", "Check knock sensor wiring. Test sensor.", "Medium"),
    "P0335": ("Crankshaft Position Sensor A Circuit", "Check CKP sensor and wiring. Replace sensor.", "Critical"),
    "P0340": ("Camshaft Position Sensor Circuit (Bank 1)", "Check CMP sensor wiring. Test sensor.", "High"),
    "P0400": ("EGR Flow Malfunction", "Clean EGR valve. Check EGR passages.", "Medium"),
    "P0420": ("Catalyst Efficiency Below Threshold (Bank 1)", "Check O2 sensors. Replace catalytic converter.", "Medium"),
    "P0430": ("Catalyst Efficiency Below Threshold (Bank 2)", "Check O2 sensors. Replace cat converter bank 2.", "Medium"),
    "P0440": ("EVAP System Malfunction", "Check gas cap. Inspect EVAP hoses.", "Low"),
    "P0442": ("EVAP Small Leak Detected", "Check gas cap seal. Smoke test EVAP system.", "Low"),
    "P0455": ("EVAP Large Leak Detected", "Check gas cap first. Smoke test system.", "Medium"),
    "P0500": ("Vehicle Speed Sensor Malfunction", "Check VSS wiring. Test sensor.", "Medium"),
    "P0505": ("Idle Control System Malfunction", "Clean throttle body and IAC valve.", "Medium"),
    "P0562": ("System Voltage Low", "Test battery and charging system. Check alternator.", "Medium"),
    "P0700": ("Transmission Control System Malfunction", "Scan transmission module for specific codes.", "High"),
    "P0741": ("Torque Converter Clutch Stuck Off", "Replace TCC solenoid. Check wiring.", "High"),
    "U0001": ("High Speed CAN Communication Bus", "Check CAN bus wiring. Look for shorts.", "Critical"),
    "U0100": ("Lost Communication with ECM/PCM", "Check ECM power and ground. Inspect CAN wiring.", "Critical"),
    "U0101": ("Lost Communication with TCM", "Check TCM power/ground. Inspect CAN wiring.", "High"),
}

# ── Serial Communication ──

def open_serial():
    global serial_fd, state
    try:
        os.system(f"stty -F {SERIAL_PORT} {BAUD_RATE} raw -echo 2>/dev/null")
        serial_fd = os.open(SERIAL_PORT, os.O_RDWR | os.O_NONBLOCK)
        state["connected"] = True
        log("Serial port opened: " + SERIAL_PORT)
        return True
    except Exception as e:
        state["connected"] = False
        log(f"Serial open failed: {e}")
        return False

def close_serial():
    global serial_fd, state
    if serial_fd is not None:
        try:
            os.close(serial_fd)
        except:
            pass
        serial_fd = None
    state["connected"] = False

def serial_write(data):
    global serial_fd
    with serial_lock:
        if serial_fd is not None:
            try:
                os.write(serial_fd, data)
                return True
            except:
                return False
    return False

def serial_read(size=4096, timeout=0.5):
    global serial_fd
    if serial_fd is None:
        return b""
    data = bytearray()
    start = time.time()
    while time.time() - start < timeout:
        try:
            chunk = os.read(serial_fd, size)
            if chunk:
                data.extend(chunk)
        except BlockingIOError:
            pass
        except:
            break
        time.sleep(0.02)
    return bytes(data)

def log(msg):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    entry = f"[{ts}] {msg}"
    state["scan_log"].append(entry)
    if len(state["scan_log"]) > 500:
        state["scan_log"] = state["scan_log"][-500:]

# ── Frame Parsing ──

def verify_checksum(frame):
    """Verify frame checksum: ~(sum of all bytes except last) & 0xFF"""
    if len(frame) < 2:
        return False
    s = sum(frame[:-1]) & 0xFF
    expected = (~s) & 0xFF
    return frame[-1] == expected

def parse_frames(data):
    """Parse CRLF-delimited MCU frames"""
    frames = []
    parts = data.split(b'\r\n')
    for part in parts:
        if len(part) >= 2:
            # Accept all frames - checksum format varies by frame type
            # 0x08 and 0x04 use ~sum checksum, 0x03 uses different format
            frames.append(bytes(part))
    return frames

def store_sensor_frame(frame):
    """Store raw 0x08 CAN relay frame with analysis"""
    if len(frame) < 9 or frame[0] != 0x08 or frame[1] != 0x8E:
        return
    state["frame_counts"]["sensor"] += 1

    hex_str = " ".join(f"{b:02X}" for b in frame)
    state["mcu_raw_bytes"].append({"hex": hex_str, "t": time.time()})
    if len(state["mcu_raw_bytes"]) > 40:
        state["mcu_raw_bytes"] = state["mcu_raw_bytes"][-40:]

    # Frame: 08 8E B2 B3 B4 B5 B6 B7 CHK
    b2, b3, b4, b5, b6, b7 = frame[2], frame[3], frame[4], frame[5], frame[6], frame[7]

    # Track per B7 group (CAN message type identifier)
    gk = f"0x{b7:02X}"
    if gk not in state["can_groups"]:
        state["can_groups"][gk] = {"count": 0, "last": None, "samples": []}
    g = state["can_groups"][gk]
    g["count"] += 1
    g["last"] = time.time()
    sample = {"b2": b2, "b3": b3, "b4": b4, "b5": b5, "b6": b6, "t": time.time()}
    g["samples"].append(sample)
    if len(g["samples"]) > 50:
        g["samples"] = g["samples"][-50:]

    # Track signal values (raw bytes for display)
    state["can_signals"] = {
        "b2": b2, "b3": b3, "b4": b4, "b5": b5, "b6": b6, "b7": b7,
        "b2h": b2 >> 4, "b4h": b4 >> 4, "b6h": b6 >> 4,
        "ts": time.time()
    }

    # History for graphs (one point per 0.5 sec)
    now = time.time()
    hist = state["can_history"]
    if not hist or now - hist[-1]["t"] >= 0.5:
        hist.append({
            "t": now,
            "b3": b3, "b5": b5, "b7": b7,
            "b2h": b2 >> 4, "b4h": b4 >> 4, "b6h": b6 >> 4,
        })
        if len(hist) > 240:
            state["can_history"] = hist[-240:]

    # Data logging if active
    if state["logging_active"]:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        state["data_log"].append(f"{ts},{b2:02X},{b3:02X},{b4:02X},{b5:02X},{b6:02X},{b7:02X}")
        if len(state["data_log"]) > 10000:
            state["data_log"] = state["data_log"][-10000:]

def process_heartbeat(frame):
    """Process type 0x04 heartbeat — ACCURATE data"""
    if len(frame) >= 5 and frame[0] == 0x04:
        # Byte 2: MIL bit + DTC count
        b2 = frame[2]
        state["mil"] = bool((b2 >> 7) & 1)
        state["dtc_count"] = b2 & 0x7F
        state["last_heartbeat"] = datetime.now().strftime("%H:%M:%S")
        state["heartbeat_count"] += 1
        state["frame_counts"]["heartbeat"] += 1
        return True
    return False

# ── Background Reader ──

def background_reader():
    global running
    fps_counter = 0
    fps_time = time.time()
    reconnect_time = 0

    while running:
        if not state["connected"]:
            if time.time() - reconnect_time > 5:
                open_serial()
                reconnect_time = time.time()
            time.sleep(1)
            continue
        try:
            data = serial_read(4096, 0.3)
            if data:
                frames = parse_frames(data)
                for frame in frames:
                    hex_str = " ".join(f"{b:02X}" for b in frame)
                    ftype = frame[0] if frame else 0

                    # Log non-CAN frames for debugging
                    if ftype != 0x08:
                        log(f"Frame type 0x{ftype:02X} ({len(frame)}b): {hex_str}")

                    if ftype == 0x04:
                        process_heartbeat(frame)
                        state["raw_frames"].append({"type": "HB", "hex": hex_str, "t": time.time()})
                    elif ftype == 0x08:
                        store_sensor_frame(frame)
                        state["raw_frames"].append({"type": "CAN", "hex": hex_str, "t": time.time()})
                    elif ftype == 0x03:
                        state["frame_counts"]["status"] += 1
                        state["raw_frames"].append({"type": "STS", "hex": hex_str, "t": time.time()})
                    else:
                        state["frame_counts"]["other"] += 1
                        state["raw_frames"].append({"type": f"0x{ftype:02X}", "hex": hex_str, "t": time.time()})

                if len(state["raw_frames"]) > 200:
                    state["raw_frames"] = state["raw_frames"][-200:]

                fps_counter += len(frames)
                now = time.time()
                if now - fps_time >= 1.0:
                    state["frames_per_sec"] = fps_counter
                    fps_counter = 0
                    fps_time = now
            else:
                time.sleep(0.05)

        except Exception as e:
            log(f"Reader error: {e}")
            time.sleep(0.5)

# ── Commands ──

def cmd_scan_dtcs():
    log("Starting DTC scan...")
    commands = [
        ("Status", bytes([0x55, 0x04, 0x50, 0x00, 0x00, 0x01, 0x51])),
        ("Warning", bytes([0x55, 0x04, 0x60, 0x00, 0x00, 0x01, 0x61])),
        ("Diag", bytes([0x55, 0x04, 0x70, 0x00, 0x00, 0x01, 0x71])),
    ]
    dtcs_found = []
    for name, cmd in commands:
        log(f"Sending {name}: {cmd.hex()}")
        serial_write(cmd)
        time.sleep(1.0)
    log(f"MCU reports {state['dtc_count']} DTCs stored. Individual codes require CAN bus service handshake.")
    log("To read individual DTCs: select Toyota > FJ Cruiser in head unit CAN bus settings")
    return dtcs_found

def cmd_clear_dtcs():
    log("Sending Clear DTCs...")
    cmd = bytes([0x55, 0x03, 0x04, 0x00, 0x01, 0x5D])
    serial_write(cmd)
    time.sleep(1.0)
    log("Clear command sent. Cycle ignition to verify.")

def cmd_raw_send(hex_str):
    try:
        data = bytes.fromhex(hex_str.replace(" ", ""))
        serial_write(data)
        log(f"Sent: {' '.join(f'{b:02X}' for b in data)}")
        time.sleep(0.5)
        resp = serial_read(4096, 1.0)
        if resp:
            frames = parse_frames(resp)
            return [" ".join(f"{b:02X}" for b in f) for f in frames]
        return []
    except Exception as e:
        log(f"Send error: {e}")
        return []

# ── HTML UI ──

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>OBD2 Pro Tuner</title>
<style>
:root {
    --bg: #08080e;
    --panel: #10101a;
    --panel2: #16162a;
    --border: #1c1c30;
    --accent: #00ff88;
    --accent2: #00ccff;
    --red: #ff3355;
    --orange: #ff8800;
    --yellow: #ffcc00;
    --text: #dde;
    --dim: #556;
    --green: #00ff88;
}
* { margin:0; padding:0; box-sizing:border-box; }
body { background: var(--bg); color: var(--text); font-family: 'Segoe UI', system-ui, sans-serif; font-size: 14px; }

/* Header */
.hdr { background: linear-gradient(180deg, #12122a 0%, #0a0a18 100%); border-bottom: 1px solid var(--accent);
  padding: 8px 12px; display: flex; align-items: center; gap: 12px; position: sticky; top: 0; z-index: 100; }
.hdr h1 { font-size: 16px; color: var(--accent); font-weight: 700; letter-spacing: 1px; }
.hdr .status { font-size: 11px; color: var(--dim); }
.hdr .dot { width: 8px; height: 8px; border-radius: 50%; display: inline-block; margin-right: 4px; }
.dot-on { background: var(--green); box-shadow: 0 0 6px var(--green); }
.dot-off { background: #333; }

/* Tabs */
.tabs { display: flex; background: var(--panel); border-bottom: 1px solid var(--border); overflow-x: auto; -webkit-overflow-scrolling: touch; }
.tabs button { background: none; border: none; color: var(--dim); padding: 10px 14px; font-size: 12px; font-weight: 600;
  cursor: pointer; white-space: nowrap; border-bottom: 2px solid transparent; font-family: inherit; }
.tabs button.active { color: var(--accent); border-bottom-color: var(--accent); }
.tabs button:hover { color: var(--text); }

/* Tab content */
.tab-content { display: none; padding: 12px; }
.tab-content.active { display: block; }

/* Cards */
.card { background: var(--panel); border: 1px solid var(--border); border-radius: 8px; padding: 12px; margin-bottom: 10px; }
.card-title { font-size: 11px; color: var(--dim); text-transform: uppercase; letter-spacing: 1px; margin-bottom: 8px; }
.card-row { display: flex; gap: 10px; flex-wrap: wrap; }

/* Gauges */
.gauge { flex: 1; min-width: 100px; text-align: center; padding: 8px; background: var(--panel2); border-radius: 6px; border: 1px solid var(--border); }
.gauge-label { font-size: 10px; color: var(--dim); text-transform: uppercase; }
.gauge-value { font-size: 28px; font-weight: 700; font-family: 'Courier New', monospace; margin: 4px 0; }
.gauge-unit { font-size: 10px; color: var(--dim); }
.gauge-bar { height: 4px; background: #1a1a30; border-radius: 2px; margin-top: 4px; overflow: hidden; }
.gauge-fill { height: 100%; border-radius: 2px; transition: width 0.3s; }

/* Status indicators */
.ind { display: inline-flex; align-items: center; gap: 6px; padding: 6px 10px; border-radius: 4px; font-size: 12px; font-weight: 600; }
.ind-ok { background: #0a2a1a; color: var(--green); border: 1px solid #0a3a2a; }
.ind-warn { background: #2a1a0a; color: var(--orange); border: 1px solid #3a2a0a; }
.ind-err { background: #2a0a0a; color: var(--red); border: 1px solid #3a0a0a; }
.ind-info { background: #0a1a2a; color: var(--accent2); border: 1px solid #0a2a3a; }

/* Raw hex display */
.hex-line { font-family: 'Courier New', monospace; font-size: 11px; padding: 2px 6px; border-bottom: 1px solid #111; }
.hex-line:nth-child(even) { background: #0c0c14; }
.hex-type { display: inline-block; width: 32px; font-weight: 700; }
.hex-type-CAN { color: var(--accent2); }
.hex-type-HB { color: var(--green); }
.hex-type-STS { color: var(--yellow); }

/* Scrollable areas */
.scroll-box { max-height: 300px; overflow-y: auto; background: #0a0a12; border-radius: 4px; border: 1px solid var(--border); }
.scroll-box-tall { max-height: 500px; }

/* Canvas */
canvas { width: 100%; background: #0a0a12; border-radius: 4px; border: 1px solid var(--border); }

/* Button */
.btn { background: var(--panel2); border: 1px solid var(--border); color: var(--text); padding: 8px 16px;
  border-radius: 4px; cursor: pointer; font-size: 12px; font-family: inherit; }
.btn:hover { border-color: var(--accent); color: var(--accent); }
.btn-accent { border-color: var(--accent); color: var(--accent); }
.btn-red { border-color: var(--red); color: var(--red); }
.btn-sm { padding: 4px 10px; font-size: 11px; }

/* Grid */
.grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.grid3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; }

/* Info table */
.info-row { display: flex; padding: 6px 0; border-bottom: 1px solid var(--border); font-size: 12px; }
.info-label { color: var(--dim); width: 140px; flex-shrink: 0; }
.info-value { color: var(--text); flex: 1; font-family: 'Courier New', monospace; }

/* Log */
.log-line { font-family: 'Courier New', monospace; font-size: 11px; padding: 1px 6px; color: #99a; }
.log-line:nth-child(even) { background: #0c0c14; }

/* MIL icon */
.mil-icon { font-size: 20px; }

/* Signal analysis grid */
.sig-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; }
.sig-cell { background: var(--panel2); padding: 6px 8px; border-radius: 4px; text-align: center; font-family: 'Courier New', monospace; }
.sig-cell .lbl { font-size: 9px; color: var(--dim); }
.sig-cell .val { font-size: 18px; font-weight: 700; }
</style>
</head>
<body>
<div class="hdr">
  <h1>OBD2 PRO TUNER</h1>
  <div style="flex:1"></div>
  <div class="status">
    <span class="dot" id="conn-dot"></span>
    <span id="conn-text">--</span>
    &nbsp;&nbsp;
    <span id="fps-text">0 fps</span>
  </div>
</div>

<div class="tabs" id="tab-bar">
  <button class="active" onclick="showTab('dash')">Dashboard</button>
  <button onclick="showTab('live')">Live Data</button>
  <button onclick="showTab('dtc')">DTCs</button>
  <button onclick="showTab('raw')">Raw Hex</button>
  <button onclick="showTab('graph')">Graphs</button>
  <button onclick="showTab('log')">Data Log</button>
  <button onclick="showTab('tools')">Tools</button>
  <button onclick="showTab('info')">Vehicle</button>
</div>

<!-- ═══ DASHBOARD ═══ -->
<div class="tab-content active" id="tab-dash">
  <div class="card">
    <div class="card-title">Engine Status</div>
    <div class="card-row">
      <div class="gauge">
        <div class="gauge-label">Check Engine</div>
        <div class="gauge-value mil-icon" id="mil-icon">--</div>
        <div class="gauge-unit" id="mil-text">--</div>
      </div>
      <div class="gauge">
        <div class="gauge-label">Fault Codes</div>
        <div class="gauge-value" id="dtc-val" style="color:var(--accent2)">--</div>
        <div class="gauge-unit">stored DTCs</div>
      </div>
      <div class="gauge">
        <div class="gauge-label">MCU Link</div>
        <div class="gauge-value" id="link-icon" style="font-size:20px">--</div>
        <div class="gauge-unit" id="link-text">--</div>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="card-title">CAN Bus Signals (Raw)</div>
    <div class="sig-grid" id="sig-grid">
      <div class="sig-cell"><div class="lbl">B2 (HI)</div><div class="val" id="sb2h">--</div></div>
      <div class="sig-cell"><div class="lbl">B3 (FULL)</div><div class="val" id="sb3">--</div></div>
      <div class="sig-cell"><div class="lbl">B4 (HI)</div><div class="val" id="sb4h">--</div></div>
      <div class="sig-cell"><div class="lbl">B5 (FULL)</div><div class="val" id="sb5">--</div></div>
      <div class="sig-cell"><div class="lbl">B6 (HI)</div><div class="val" id="sb6h">--</div></div>
      <div class="sig-cell"><div class="lbl">B7 (TYPE)</div><div class="val" id="sb7">--</div></div>
    </div>
  </div>

  <div class="card">
    <div class="card-title">Signal Waveform</div>
    <canvas id="dash-canvas" height="120"></canvas>
  </div>

  <div class="card">
    <div class="card-title">Connection Info</div>
    <div style="display:flex;gap:8px;flex-wrap:wrap">
      <div class="ind" id="ind-conn">--</div>
      <div class="ind" id="ind-rate">--</div>
      <div class="ind" id="ind-frames">--</div>
      <div class="ind" id="ind-uptime">--</div>
    </div>
  </div>
</div>

<!-- ═══ LIVE DATA ═══ -->
<div class="tab-content" id="tab-live">
  <div class="card">
    <div class="card-title">CAN Message Groups</div>
    <div class="scroll-box" id="live-groups">Loading...</div>
  </div>

  <div class="card">
    <div class="card-title">Byte Analysis (All Frames)</div>
    <div class="sig-grid">
      <div class="sig-cell"><div class="lbl">B2 Range</div><div class="val" id="lb2r">--</div></div>
      <div class="sig-cell"><div class="lbl">B3 Range</div><div class="val" id="lb3r">--</div></div>
      <div class="sig-cell"><div class="lbl">B4 Range</div><div class="val" id="lb4r">--</div></div>
      <div class="sig-cell"><div class="lbl">B5 Range</div><div class="val" id="lb5r">--</div></div>
      <div class="sig-cell"><div class="lbl">B6 Range</div><div class="val" id="lb6r">--</div></div>
      <div class="sig-cell"><div class="lbl">Groups</div><div class="val" id="lgrp">--</div></div>
    </div>
  </div>

  <div class="card">
    <div class="card-title">Frame Type Distribution</div>
    <div id="frame-dist"></div>
  </div>
</div>

<!-- ═══ DTCs ═══ -->
<div class="tab-content" id="tab-dtc">
  <div class="card">
    <div class="card-title">Diagnostic Trouble Codes</div>
    <div style="display:flex;gap:8px;margin-bottom:10px">
      <button class="btn btn-accent" onclick="scanDTCs()">Scan DTCs</button>
      <button class="btn btn-red" onclick="clearDTCs()">Clear DTCs</button>
    </div>
    <div id="dtc-status"></div>
    <div id="dtc-list"></div>
  </div>

  <div class="card">
    <div class="card-title">DTC Notes</div>
    <div style="font-size:12px;color:var(--dim);line-height:1.6">
      MCU heartbeat confirms <strong id="dtc-note-count">0</strong> stored DTCs.<br>
      Individual DTC codes require the CAN bus service to be active.<br>
      To activate: go to head unit Settings > CAN Bus > Toyota > FJ Cruiser.<br>
      Once activated, the head unit's CAN parser (LuZhengCanParseToyotaFJ) will decode all parameters.
    </div>
  </div>
</div>

<!-- ═══ RAW HEX ═══ -->
<div class="tab-content" id="tab-raw">
  <div class="card">
    <div class="card-title">Raw Serial Frames (Live)</div>
    <div style="margin-bottom:8px;display:flex;gap:8px">
      <button class="btn btn-sm" id="raw-pause-btn" onclick="toggleRawPause()">Pause</button>
      <button class="btn btn-sm" onclick="clearRaw()">Clear</button>
      <span style="font-size:11px;color:var(--dim);padding-top:6px" id="raw-count">0 frames</span>
    </div>
    <div class="scroll-box scroll-box-tall" id="raw-hex">Loading...</div>
  </div>

  <div class="card">
    <div class="card-title">Frame Format Reference</div>
    <div style="font-size:11px;color:var(--dim);line-height:1.6;font-family:'Courier New',monospace">
      Delimiter: 0D 0A (CRLF)<br>
      Heartbeat: 04 71 [STATUS] [B3] [CHK]<br>
      &nbsp; STATUS bit7=MIL, bits0-6=DTC count<br>
      CAN Relay: 08 8E [B2] [B3] [B4] [B5] [B6] [B7] [CHK]<br>
      &nbsp; B2,B4,B6: data in HIGH nibble only (low=0)<br>
      &nbsp; B3,B5: full byte values<br>
      &nbsp; B7: CAN message type identifier<br>
      Checksum: ~(sum of all bytes except last) & 0xFF
    </div>
  </div>
</div>

<!-- ═══ GRAPHS ═══ -->
<div class="tab-content" id="tab-graph">
  <div class="card">
    <div class="card-title">B3 Signal (Full Byte) — Time Series</div>
    <canvas id="graph-b3" height="150"></canvas>
  </div>
  <div class="card">
    <div class="card-title">B5 Signal (Full Byte) — Time Series</div>
    <canvas id="graph-b5" height="150"></canvas>
  </div>
  <div class="card">
    <div class="card-title">High Nibbles (B2h, B4h, B6h) — Overlay</div>
    <canvas id="graph-nibbles" height="150"></canvas>
  </div>
  <div class="card">
    <div class="card-title">B7 Message Type — Distribution</div>
    <canvas id="graph-b7" height="100"></canvas>
  </div>
</div>

<!-- ═══ DATA LOG ═══ -->
<div class="tab-content" id="tab-log">
  <div class="card">
    <div class="card-title">Data Logging</div>
    <div style="display:flex;gap:8px;margin-bottom:10px">
      <button class="btn" id="log-toggle-btn" onclick="toggleLogging()">Start Logging</button>
      <button class="btn btn-sm" onclick="downloadLog()">Download CSV</button>
      <button class="btn btn-sm btn-red" onclick="clearLog()">Clear</button>
    </div>
    <div id="log-status" style="font-size:12px;color:var(--dim);margin-bottom:8px">Logging: OFF</div>
    <div class="scroll-box scroll-box-tall" id="log-data" style="font-family:'Courier New',monospace;font-size:11px">
      Timestamp,B2,B3,B4,B5,B6,B7
    </div>
  </div>
</div>

<!-- ═══ TOOLS ═══ -->
<div class="tab-content" id="tab-tools">
  <div class="card">
    <div class="card-title">Send Raw Command</div>
    <div style="display:flex;gap:8px;margin-bottom:8px">
      <input type="text" id="raw-cmd" placeholder="55 04 50 00 00 01 51" style="flex:1;background:var(--panel2);border:1px solid var(--border);color:var(--text);padding:8px;border-radius:4px;font-family:'Courier New',monospace;font-size:12px">
      <button class="btn btn-accent" onclick="sendRawCmd()">Send</button>
    </div>
    <div class="scroll-box" id="cmd-response" style="min-height:60px"></div>
  </div>

  <div class="card">
    <div class="card-title">Quick Commands</div>
    <div style="display:flex;gap:6px;flex-wrap:wrap">
      <button class="btn btn-sm" onclick="sendQuick('55 04 50 00 00 01 51')">Status 0x50</button>
      <button class="btn btn-sm" onclick="sendQuick('55 04 60 00 00 01 61')">Warning 0x60</button>
      <button class="btn btn-sm" onclick="sendQuick('55 04 70 00 00 01 71')">Diag 0x70</button>
      <button class="btn btn-sm" onclick="sendQuick('55 04 80 00 00 01 81')">VIN 0x80</button>
      <button class="btn btn-sm" onclick="sendQuick('55 03 01 00 01 59')">Ping MCU</button>
    </div>
  </div>

  <div class="card">
    <div class="card-title">System Log</div>
    <div class="scroll-box scroll-box-tall" id="sys-log"></div>
  </div>
</div>

<!-- ═══ VEHICLE INFO ═══ -->
<div class="tab-content" id="tab-info">
  <div class="card">
    <div class="card-title">Vehicle Information</div>
    <div id="vehicle-info"></div>
  </div>
  <div class="card">
    <div class="card-title">CAN Bus Architecture</div>
    <div style="font-size:12px;color:var(--dim);line-height:1.8">
      <div class="info-row"><div class="info-label">CAN Adapter</div><div class="info-value">LuZheng (Szchoiceway)</div></div>
      <div class="info-row"><div class="info-label">Parser Class</div><div class="info-value">LuZhengCanParseToyotaFJ</div></div>
      <div class="info-row"><div class="info-label">Serial Protocol</div><div class="info-value">CRLF-delimited binary, 115200 baud</div></div>
      <div class="info-row"><div class="info-label">CAN Frame Format</div><div class="info-value">2E 2E LEN [CMD_ID] [DATA...] CHK</div></div>
      <div class="info-row"><div class="info-label">MCU Frame Format</div><div class="info-value">0D 0A [TYPE] [DATA...] [CHK]</div></div>
      <div class="info-row"><div class="info-label">Speed Formula</div><div class="info-value">((B3&0xFF)+(B4&0xFF)*256)/16</div></div>
      <div class="info-row"><div class="info-label">RPM Formula</div><div class="info-value">B4(high) + B3(low) from cmd 0x50</div></div>
      <div class="info-row"><div class="info-label">CAN Service</div><div class="info-value" id="can-svc-status">Requires activation</div></div>
    </div>
  </div>
  <div class="card">
    <div class="card-title">About</div>
    <div style="font-size:12px;color:var(--dim);line-height:1.6">
      MobileCLI OBD2 Pro Tuner v2.0<br>
      Built with MobileCLI on Android<br>
      github.com/MobileDevCLI/MobileCLI-Vehicle
    </div>
  </div>
</div>

<script>
let rawPaused = false;
let updateTimer = null;

function showTab(id) {
  document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
  document.querySelectorAll('.tabs button').forEach(el => el.classList.remove('active'));
  document.getElementById('tab-'+id).classList.add('active');
  const btns = document.querySelectorAll('.tabs button');
  const tabs = ['dash','live','dtc','raw','graph','log','tools','info'];
  const idx = tabs.indexOf(id);
  if (idx >= 0 && btns[idx]) btns[idx].classList.add('active');
}

function updateUI(d) {
  // Connection
  const dot = document.getElementById('conn-dot');
  const ct = document.getElementById('conn-text');
  if (d.connected) { dot.className='dot dot-on'; ct.textContent='CONNECTED'; }
  else { dot.className='dot dot-off'; ct.textContent='DISCONNECTED'; }
  document.getElementById('fps-text').textContent = d.frames_per_sec + ' fps';

  // MIL
  const mi = document.getElementById('mil-icon');
  const mt = document.getElementById('mil-text');
  if (d.mil) { mi.textContent='ON'; mi.style.color='var(--red)'; mt.textContent='MIL ACTIVE'; mt.style.color='var(--red)'; }
  else if (d.last_heartbeat) { mi.textContent='OFF'; mi.style.color='var(--green)'; mt.textContent='No MIL'; mt.style.color='var(--green)'; }
  else { mi.textContent='--'; mi.style.color='var(--yellow)'; mt.textContent='Awaiting heartbeat'; mt.style.color='var(--dim)'; }

  // DTCs
  const dv = document.getElementById('dtc-val');
  dv.textContent = d.dtc_count;
  dv.style.color = d.dtc_count > 0 ? 'var(--red)' : 'var(--green)';
  document.getElementById('dtc-note-count').textContent = d.dtc_count;

  // Link
  const li = document.getElementById('link-icon');
  const lt = document.getElementById('link-text');
  if (d.connected && d.last_heartbeat) {
    li.textContent='LIVE'; li.style.color='var(--green)';
    lt.textContent='HB: ' + d.last_heartbeat;
  } else if (d.connected) {
    li.textContent='WAIT'; li.style.color='var(--yellow)';
    lt.textContent='Waiting for heartbeat';
  } else {
    li.textContent='OFF'; li.style.color='var(--red)';
    lt.textContent='No connection';
  }

  // CAN Signals
  const sig = d.can_signals || {};
  if (sig.ts) {
    document.getElementById('sb2h').textContent = sig.b2h !== undefined ? sig.b2h : '--';
    document.getElementById('sb3').textContent = sig.b3 !== undefined ? '0x'+(sig.b3).toString(16).toUpperCase().padStart(2,'0') : '--';
    document.getElementById('sb4h').textContent = sig.b4h !== undefined ? sig.b4h : '--';
    document.getElementById('sb5').textContent = sig.b5 !== undefined ? '0x'+(sig.b5).toString(16).toUpperCase().padStart(2,'0') : '--';
    document.getElementById('sb6h').textContent = sig.b6h !== undefined ? sig.b6h : '--';
    document.getElementById('sb7').textContent = sig.b7 !== undefined ? '0x'+(sig.b7).toString(16).toUpperCase().padStart(2,'0') : '--';
  }

  // Connection indicators
  const ic = document.getElementById('ind-conn');
  ic.className = 'ind ' + (d.connected ? 'ind-ok' : 'ind-err');
  ic.textContent = d.connected ? 'Serial OK' : 'No Serial';
  const ir = document.getElementById('ind-rate');
  ir.className = 'ind ind-info';
  ir.textContent = d.frames_per_sec + ' fps';
  const ifr = document.getElementById('ind-frames');
  ifr.className = 'ind ind-info';
  ifr.textContent = (d.frame_counts.sensor + d.frame_counts.heartbeat) + ' total';
  const iu = document.getElementById('ind-uptime');
  iu.className = 'ind ind-info';
  const up = Math.floor((Date.now()/1000) - d.start_time);
  const um = Math.floor(up/60), us = up%60;
  iu.textContent = um + 'm ' + us + 's';

  // Dashboard waveform
  drawDashWaveform(d.can_history || []);

  // Live data tab
  updateLiveData(d);

  // Raw hex tab
  if (!rawPaused) updateRawHex(d.raw_frames || []);

  // Graphs
  drawGraphs(d.can_history || []);

  // Frame distribution
  updateFrameDist(d.frame_counts);

  // Vehicle info
  updateVehicleInfo(d);

  // System log
  updateSysLog(d.scan_log || []);

  // Data log
  updateDataLog(d);
}

function drawDashWaveform(hist) {
  const c = document.getElementById('dash-canvas');
  if (!c) return;
  const ctx = c.getContext('2d');
  c.width = c.offsetWidth;
  const W = c.width, H = c.height;
  ctx.fillStyle = '#0a0a12';
  ctx.fillRect(0, 0, W, H);

  if (hist.length < 2) { ctx.fillStyle='#333'; ctx.fillText('Waiting for data...', W/2-50, H/2); return; }

  // Draw grid
  ctx.strokeStyle = '#1a1a2a';
  ctx.lineWidth = 0.5;
  for (let y = 0; y < H; y += 20) { ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke(); }

  const n = Math.min(hist.length, 120);
  const data = hist.slice(-n);
  const dx = W / (n - 1);

  // B3 signal (green)
  drawLine(ctx, data, 'b3', 0, 255, '#00ff88', dx, H);
  // B5 signal (blue)
  drawLine(ctx, data, 'b5', 0, 255, '#00ccff', dx, H);
  // B2h nibble (orange, scaled)
  drawLine(ctx, data, 'b2h', 0, 15, '#ff8800', dx, H);

  // Legend
  ctx.font = '10px sans-serif';
  ctx.fillStyle = '#00ff88'; ctx.fillText('B3', 5, 12);
  ctx.fillStyle = '#00ccff'; ctx.fillText('B5', 30, 12);
  ctx.fillStyle = '#ff8800'; ctx.fillText('B2h', 55, 12);
}

function drawLine(ctx, data, key, min, max, color, dx, H) {
  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  for (let i = 0; i < data.length; i++) {
    const v = data[i][key];
    if (v === undefined) continue;
    const y = H - ((v - min) / (max - min)) * (H - 20) - 10;
    if (i === 0) ctx.moveTo(0, y);
    else ctx.lineTo(i * dx, y);
  }
  ctx.stroke();
}

function updateLiveData(d) {
  const el = document.getElementById('live-groups');
  if (!el) return;
  const groups = d.can_groups || {};
  const keys = Object.keys(groups).sort();
  if (keys.length === 0) { el.innerHTML = '<div style="padding:10px;color:var(--dim)">Waiting for CAN data...</div>'; return; }

  let html = '<table style="width:100%;font-size:11px;font-family:monospace;border-collapse:collapse">';
  html += '<tr style="color:var(--dim)"><th style="text-align:left;padding:4px">Type</th><th>Count</th><th>Last B2-B6</th><th>Rate</th></tr>';
  for (const k of keys) {
    const g = groups[k];
    const s = g.samples && g.samples.length > 0 ? g.samples[g.samples.length-1] : null;
    const lastData = s ? `${s.b2.toString(16).padStart(2,'0')} ${s.b3.toString(16).padStart(2,'0')} ${s.b4.toString(16).padStart(2,'0')} ${s.b5.toString(16).padStart(2,'0')} ${s.b6.toString(16).padStart(2,'0')}`.toUpperCase() : '--';
    const ago = g.last ? Math.round((Date.now()/1000 - g.last)*10)/10 : '--';
    html += `<tr style="border-top:1px solid #1a1a2a"><td style="padding:4px;color:var(--accent2)">${k}</td><td style="text-align:center">${g.count}</td><td style="color:var(--text)">${lastData}</td><td style="text-align:center;color:var(--dim)">${ago}s ago</td></tr>`;
  }
  html += '</table>';
  el.innerHTML = html;

  // Byte ranges
  const hist = d.can_history || [];
  if (hist.length > 0) {
    const last20 = hist.slice(-20);
    const r = (arr, k) => { const vs = arr.map(x=>x[k]).filter(v=>v!==undefined); return vs.length ? (Math.min(...vs).toString(16)+'-'+Math.max(...vs).toString(16)).toUpperCase() : '--'; };
    document.getElementById('lb2r').textContent = r(last20,'b2h');
    document.getElementById('lb3r').textContent = r(last20,'b3');
    document.getElementById('lb5r').textContent = r(last20,'b5');
    document.getElementById('lb4r').textContent = r(last20,'b4h');
    document.getElementById('lb6r').textContent = r(last20,'b6h');
    document.getElementById('lgrp').textContent = keys.length;
  }
}

function updateRawHex(frames) {
  const el = document.getElementById('raw-hex');
  if (!el) return;
  const last = frames.slice(-50).reverse();
  document.getElementById('raw-count').textContent = frames.length + ' frames';
  let html = '';
  for (const f of last) {
    html += `<div class="hex-line"><span class="hex-type hex-type-${f.type}">${f.type}</span> ${f.hex}</div>`;
  }
  el.innerHTML = html || '<div style="padding:10px;color:var(--dim)">No frames yet</div>';
}

function toggleRawPause() {
  rawPaused = !rawPaused;
  document.getElementById('raw-pause-btn').textContent = rawPaused ? 'Resume' : 'Pause';
  document.getElementById('raw-pause-btn').className = rawPaused ? 'btn btn-sm btn-accent' : 'btn btn-sm';
}

function clearRaw() { fetch('/api/cmd?action=clear_raw'); }

function drawGraphs(hist) {
  if (hist.length < 2) return;
  const n = Math.min(hist.length, 120);
  const data = hist.slice(-n);

  // B3 graph
  drawTimeGraph('graph-b3', data, [{key:'b3', color:'#00ff88', label:'B3', min:0, max:255}]);
  // B5 graph
  drawTimeGraph('graph-b5', data, [{key:'b5', color:'#00ccff', label:'B5', min:0, max:255}]);
  // Nibbles overlay
  drawTimeGraph('graph-nibbles', data, [
    {key:'b2h', color:'#ff8800', label:'B2h', min:0, max:15},
    {key:'b4h', color:'#00ff88', label:'B4h', min:0, max:15},
    {key:'b6h', color:'#00ccff', label:'B6h', min:0, max:15},
  ]);
  // B7 distribution
  drawB7Dist('graph-b7', data);
}

function drawTimeGraph(id, data, series) {
  const c = document.getElementById(id);
  if (!c) return;
  const ctx = c.getContext('2d');
  c.width = c.offsetWidth;
  const W = c.width, H = c.height;
  ctx.fillStyle = '#0a0a12';
  ctx.fillRect(0, 0, W, H);
  // Grid
  ctx.strokeStyle = '#1a1a2a'; ctx.lineWidth = 0.5;
  for (let y = 0; y < H; y += 30) { ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke(); }
  const dx = W / (data.length - 1);
  // Series
  let lx = 5;
  for (const s of series) {
    drawLine(ctx, data, s.key, s.min, s.max, s.color, dx, H);
    ctx.font = '10px sans-serif';
    ctx.fillStyle = s.color;
    ctx.fillText(s.label, lx, 12);
    lx += ctx.measureText(s.label).width + 10;
  }
  // Y-axis labels
  ctx.font = '9px monospace'; ctx.fillStyle = '#444';
  if (series.length === 1) {
    ctx.fillText(series[0].max, W-25, 12);
    ctx.fillText(series[0].min, W-15, H-4);
  }
}

function drawB7Dist(id, data) {
  const c = document.getElementById(id);
  if (!c) return;
  const ctx = c.getContext('2d');
  c.width = c.offsetWidth;
  const W = c.width, H = c.height;
  ctx.fillStyle = '#0a0a12';
  ctx.fillRect(0, 0, W, H);
  // Count B7 values
  const counts = {};
  for (const d of data) { const k = '0x'+(d.b7||0).toString(16).toUpperCase().padStart(2,'0'); counts[k] = (counts[k]||0)+1; }
  const keys = Object.keys(counts).sort();
  if (keys.length === 0) return;
  const max = Math.max(...Object.values(counts));
  const bw = Math.min(60, (W-20) / keys.length - 4);
  const colors = ['#00ff88','#00ccff','#ff8800','#ffcc00','#ff3355','#aa66ff','#66ffaa','#ff66aa'];
  let x = 20;
  for (let i = 0; i < keys.length; i++) {
    const h = (counts[keys[i]] / max) * (H - 30);
    ctx.fillStyle = colors[i % colors.length];
    ctx.fillRect(x, H - h - 15, bw, h);
    ctx.font = '9px monospace'; ctx.fillStyle = '#888';
    ctx.fillText(keys[i], x, H - 2);
    ctx.fillStyle = colors[i % colors.length];
    ctx.fillText(counts[keys[i]], x, H - h - 18);
    x += bw + 4;
  }
}

function updateFrameDist(fc) {
  const el = document.getElementById('frame-dist');
  if (!el) return;
  const total = Object.values(fc).reduce((a,b)=>a+b, 0) || 1;
  let html = '';
  const colors = {sensor:'var(--accent2)',heartbeat:'var(--green)',status:'var(--yellow)',other:'var(--dim)'};
  for (const [k,v] of Object.entries(fc)) {
    const pct = (v/total*100).toFixed(1);
    html += `<div style="display:flex;align-items:center;gap:8px;margin:4px 0;font-size:12px">
      <span style="width:70px;color:${colors[k]||'var(--dim)'}">${k}</span>
      <div style="flex:1;height:14px;background:#111;border-radius:2px;overflow:hidden">
        <div style="width:${pct}%;height:100%;background:${colors[k]||'#333'}"></div>
      </div>
      <span style="width:60px;text-align:right;color:var(--dim)">${v} (${pct}%)</span>
    </div>`;
  }
  el.innerHTML = html;
}

function updateVehicleInfo(d) {
  const el = document.getElementById('vehicle-info');
  if (!el) return;
  const fields = [
    ['Vehicle', d.vehicle], ['VIN', d.vin], ['Protocol', d.protocol],
    ['CAN Profile', d.canbus_profile], ['Serial Port', d.port],
    ['Baud Rate', d.baud], ['Head Unit', d.head_unit],
    ['Platform', d.platform], ['Firmware', d.firmware],
    ['MCU ID', d.mcu_id], ['Heartbeats', d.heartbeat_count],
    ['MIL Status', d.mil ? 'ON' : 'OFF'], ['DTC Count', d.dtc_count],
  ];
  let html = '';
  for (const [l,v] of fields) {
    html += `<div class="info-row"><div class="info-label">${l}</div><div class="info-value">${v}</div></div>`;
  }
  el.innerHTML = html;
}

function updateSysLog(logs) {
  const el = document.getElementById('sys-log');
  if (!el) return;
  const last = logs.slice(-100);
  el.innerHTML = last.map(l => `<div class="log-line">${l}</div>`).join('');
  el.scrollTop = el.scrollHeight;
}

function updateDataLog(d) {
  const el = document.getElementById('log-data');
  if (!el) return;
  const logBtn = document.getElementById('log-toggle-btn');
  const logStatus = document.getElementById('log-status');
  if (d.logging_active) {
    logBtn.textContent = 'Stop Logging';
    logBtn.className = 'btn btn-red';
    logStatus.textContent = 'Logging: ON (' + (d.data_log||[]).length + ' entries)';
    logStatus.style.color = 'var(--green)';
  } else {
    logBtn.textContent = 'Start Logging';
    logBtn.className = 'btn btn-accent';
    logStatus.textContent = 'Logging: OFF';
    logStatus.style.color = 'var(--dim)';
  }
  const log = d.data_log || [];
  const last = log.slice(-50);
  el.innerHTML = 'Timestamp,B2,B3,B4,B5,B6,B7\n' + last.join('\n');
  el.scrollTop = el.scrollHeight;
}

// API calls
function fetchState() {
  fetch('/api/state').then(r=>r.json()).then(d=>updateUI(d)).catch(()=>{});
}

function scanDTCs() {
  document.getElementById('dtc-status').innerHTML = '<div class="ind ind-info">Scanning...</div>';
  fetch('/api/cmd?action=scan_dtcs').then(r=>r.json()).then(d => {
    document.getElementById('dtc-status').innerHTML = '<div class="ind ind-ok">Scan complete</div>';
  });
}

function clearDTCs() {
  if (confirm('Clear all stored DTCs? This will turn off the check engine light.')) {
    fetch('/api/cmd?action=clear_dtcs').then(r=>r.json());
  }
}

function sendRawCmd() {
  const cmd = document.getElementById('raw-cmd').value.trim();
  if (!cmd) return;
  fetch('/api/cmd?action=raw_send&data=' + encodeURIComponent(cmd)).then(r=>r.json()).then(d => {
    const el = document.getElementById('cmd-response');
    const resp = d.response || [];
    el.innerHTML = resp.length > 0
      ? resp.map(r => `<div class="hex-line">${r}</div>`).join('')
      : '<div style="padding:6px;color:var(--dim)">No response (MCU may not respond without CAN handshake)</div>';
  });
}

function sendQuick(cmd) {
  document.getElementById('raw-cmd').value = cmd;
  sendRawCmd();
}

function toggleLogging() {
  fetch('/api/cmd?action=toggle_logging').then(r=>r.json());
}

function downloadLog() {
  fetch('/api/log_csv').then(r=>r.text()).then(csv => {
    const blob = new Blob([csv], {type:'text/csv'});
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'obd2_log_' + new Date().toISOString().slice(0,19).replace(/:/g,'') + '.csv';
    a.click();
  });
}

function clearLog() { fetch('/api/cmd?action=clear_log'); }

// Start polling
updateTimer = setInterval(fetchState, 500);
fetchState();
</script>
</body>
</html>"""


# ── HTTP Server ──

class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress default logging

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())

        elif path == "/api/state":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            # Build safe response (avoid sending huge sample arrays)
            resp = dict(state)
            groups = {}
            for k, v in state["can_groups"].items():
                last_sample = v["samples"][-1] if v["samples"] else None
                groups[k] = {"count": v["count"], "last": v["last"], "samples": [last_sample] if last_sample else []}
            resp["can_groups"] = groups
            resp["can_history"] = state["can_history"][-120:]
            resp["data_log"] = state["data_log"][-100:]
            resp["raw_frames"] = state["raw_frames"][-50:]
            resp["scan_log"] = state["scan_log"][-100:]
            self.wfile.write(json.dumps(resp, default=str).encode())

        elif path == "/api/cmd":
            params = parse_qs(parsed.query)
            action = params.get("action", [""])[0]
            result = {"ok": True}

            if action == "scan_dtcs":
                threading.Thread(target=cmd_scan_dtcs, daemon=True).start()
            elif action == "clear_dtcs":
                threading.Thread(target=cmd_clear_dtcs, daemon=True).start()
            elif action == "raw_send":
                data = params.get("data", [""])[0]
                resp = cmd_raw_send(data)
                result["response"] = resp
            elif action == "toggle_logging":
                state["logging_active"] = not state["logging_active"]
                log(f"Data logging {'started' if state['logging_active'] else 'stopped'}")
            elif action == "clear_log":
                state["data_log"] = []
                log("Data log cleared")
            elif action == "clear_raw":
                state["raw_frames"] = []

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())

        elif path == "/api/log_csv":
            self.send_response(200)
            self.send_header("Content-Type", "text/csv")
            self.send_header("Content-Disposition", "attachment; filename=obd2_log.csv")
            self.end_headers()
            csv = "Timestamp,B2,B3,B4,B5,B6,B7\n"
            csv += "\n".join(state["data_log"])
            self.wfile.write(csv.encode())

        else:
            self.send_response(404)
            self.end_headers()

# ── Main ──

def main():
    global running

    print(f"\n  OBD2 Pro Tuner v2.0")
    print(f"  Serial: {SERIAL_PORT} @ {BAUD_RATE}")
    print(f"  Web UI: http://localhost:{PORT}")
    print(f"  Press Ctrl+C to stop\n")

    open_serial()

    reader_thread = threading.Thread(target=background_reader, daemon=True)
    reader_thread.start()

    server = HTTPServer(("127.0.0.1", PORT), Handler)
    server.socket.settimeout(1)

    try:
        while running:
            server.handle_request()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        running = False
        close_serial()
        server.server_close()

if __name__ == "__main__":
    main()

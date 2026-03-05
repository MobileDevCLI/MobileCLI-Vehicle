#!/usr/bin/env python3
"""
MobileCLI OBD2 Pro Tuner — Web-based vehicle diagnostics & tuning interface
Runs on localhost:8099, communicates with /dev/ttyHS1 serial MCU
"""

import http.server
import json
import os
import sys
import time
import threading
import struct
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

SERIAL_PORT = "/dev/ttyHS1"
BAUD_RATE = 115200
PORT = 8099

# ─── Global State ──────────────────────────────────────────
state = {
    "connected": False,
    "mil": False,
    "dtc_count": 0,
    "dtcs": [],
    "last_heartbeat": None,
    "heartbeat_count": 0,
    "raw_frames": [],
    "scan_log": [],
    "frame_counts": {"sensor": 0, "heartbeat": 0, "extended": 0, "status": 0, "ident": 0, "other": 0},
    "frames_per_sec": 0,
    "mcu_raw_bytes": [],  # Last 8 raw 0x08 frame payloads for display
    "vin": "Not available",
    "vehicle": "2008 Toyota FJ Cruiser 4WD",
    "protocol": "Szchoiceway LuZheng (Toyota FJ)",
    "canbus_profile": "CARTYPE_TOYOTA_FJ_CRUISER",
    "canbus_service": "NOT RUNNING",
    "port": SERIAL_PORT,
    "baud": BAUD_RATE,
    "firmware": "GT6-EAU-T16.00.050",
    "mcu_id": "PRLC0__GT6E_GB___S148 7808",
    "head_unit": "Szchoiceway GT6-CAR",
    "platform": "Qualcomm SM6125 / Android 13",
}

serial_fd = None
serial_lock = threading.Lock()
running = True

# ─── DTC Database ──────────────────────────────────────────
DTC_DB = {
    "P0010": ("Intake Camshaft Position Actuator Circuit (Bank 1)", "Check wiring to camshaft position actuator. Inspect actuator solenoid. Check oil level and condition.", "Medium"),
    "P0011": ("Intake Cam Position Timing Over-Advanced (Bank 1)", "Check engine oil level/condition. Replace camshaft position actuator solenoid. Check timing chain stretch.", "High"),
    "P0012": ("Intake Cam Position Timing Over-Retarded (Bank 1)", "Check engine oil. Inspect VVT solenoid and screen. Check timing chain.", "High"),
    "P0013": ("Exhaust Camshaft Position Actuator Circuit (Bank 1)", "Check wiring and connector. Test actuator solenoid resistance. Replace if faulty.", "Medium"),
    "P0014": ("Exhaust Cam Position Timing Over-Advanced (Bank 1)", "Change engine oil. Inspect VVT system. Check for sludge buildup.", "High"),
    "P0015": ("Exhaust Cam Position Timing Over-Retarded (Bank 1)", "Check oil level. Inspect VVT solenoid. Check timing components.", "High"),
    "P0016": ("Crank/Cam Position Correlation Bank 1 Sensor A", "Check timing chain/belt alignment. Inspect tone rings. Check sensors.", "Critical"),
    "P0017": ("Crank/Cam Position Correlation Bank 1 Sensor B", "Check timing chain/belt. Inspect reluctor ring. Test sensors.", "Critical"),
    "P0100": ("MAF Circuit Malfunction", "Clean MAF sensor. Check wiring. Replace MAF if cleaning fails.", "High"),
    "P0101": ("MAF Circuit Range/Performance", "Clean MAF sensor with MAF cleaner spray. Check for air leaks.", "Medium"),
    "P0102": ("MAF Circuit Low Input", "Check MAF connector. Inspect wiring for damage. Test MAF sensor.", "Medium"),
    "P0103": ("MAF Circuit High Input", "Check for short in MAF circuit. Inspect connector. Replace MAF.", "Medium"),
    "P0106": ("MAP/Barometric Pressure Range/Performance", "Check vacuum hoses. Test MAP sensor. Check for intake leaks.", "Medium"),
    "P0107": ("MAP/Barometric Pressure Circuit Low", "Check MAP sensor wiring. Test sensor voltage. Replace if bad.", "Medium"),
    "P0108": ("MAP/Barometric Pressure Circuit High", "Check for short to voltage. Test MAP sensor. Inspect wiring.", "Medium"),
    "P0110": ("Intake Air Temperature Circuit Malfunction", "Check IAT sensor connector. Test sensor resistance. Replace.", "Low"),
    "P0111": ("Intake Air Temperature Range/Performance", "Check IAT sensor mounting. Compare with ambient temp. Replace sensor.", "Low"),
    "P0112": ("Intake Air Temperature Circuit Low", "Check for short to ground. Test IAT sensor. Inspect wiring.", "Low"),
    "P0113": ("Intake Air Temperature Circuit High", "Check IAT connector. Test for open circuit. Replace sensor.", "Low"),
    "P0115": ("Engine Coolant Temperature Circuit", "Check ECT sensor connector. Test sensor resistance. Replace.", "Medium"),
    "P0116": ("Engine Coolant Temperature Range/Performance", "Check thermostat operation. Test ECT sensor. Check cooling system.", "Medium"),
    "P0117": ("Engine Coolant Temperature Circuit Low", "Check for short to ground in ECT circuit. Test sensor.", "Medium"),
    "P0118": ("Engine Coolant Temperature Circuit High", "Check ECT connector for corrosion. Test for open circuit.", "Medium"),
    "P0120": ("Throttle Position Sensor Circuit", "Check TPS wiring. Test TPS voltage sweep. Replace TPS.", "High"),
    "P0121": ("Throttle Position Sensor Range/Performance", "Clean throttle body. Check TPS. Inspect wiring.", "High"),
    "P0122": ("Throttle Position Sensor Circuit Low", "Check TPS ground circuit. Test sensor. Inspect connector.", "High"),
    "P0123": ("Throttle Position Sensor Circuit High", "Check for short to voltage. Test TPS. Replace if faulty.", "High"),
    "P0125": ("Insufficient Coolant Temp for Closed Loop", "Check thermostat. Verify ECT sensor. Check cooling system.", "Low"),
    "P0128": ("Coolant Thermostat Below Regulating Temp", "Replace thermostat. Check ECT sensor accuracy.", "Medium"),
    "P0130": ("O2 Sensor Circuit (Bank 1 Sensor 1)", "Test O2 sensor voltage. Check wiring. Replace O2 sensor.", "Medium"),
    "P0131": ("O2 Sensor Low Voltage (B1S1)", "Check for vacuum leaks. Test O2 sensor. Check fuel pressure.", "Medium"),
    "P0132": ("O2 Sensor High Voltage (B1S1)", "Check for rich condition. Inspect fuel injectors. Test O2.", "Medium"),
    "P0133": ("O2 Sensor Slow Response (B1S1)", "Replace O2 sensor. Check for exhaust leaks before sensor.", "Medium"),
    "P0134": ("O2 Sensor No Activity (B1S1)", "Check O2 heater circuit. Test sensor. Check wiring.", "Medium"),
    "P0135": ("O2 Sensor Heater Circuit (B1S1)", "Check O2 heater fuse. Test heater resistance. Replace O2.", "Medium"),
    "P0136": ("O2 Sensor Circuit (Bank 1 Sensor 2)", "Test downstream O2 sensor. Check wiring. Replace sensor.", "Medium"),
    "P0137": ("O2 Sensor Low Voltage (B1S2)", "Check for exhaust leaks. Test O2 sensor. Inspect catalyst.", "Medium"),
    "P0138": ("O2 Sensor High Voltage (B1S2)", "Check for rich condition. Inspect catalyst. Test O2 sensor.", "Medium"),
    "P0171": ("System Too Lean (Bank 1)", "Check for vacuum/intake leaks. Clean MAF. Check fuel pressure. Inspect injectors.", "High"),
    "P0172": ("System Too Rich (Bank 1)", "Check for leaking injectors. Inspect fuel pressure regulator. Clean MAF.", "High"),
    "P0174": ("System Too Lean (Bank 2)", "Check for vacuum leaks on bank 2. Clean MAF. Check fuel delivery.", "High"),
    "P0175": ("System Too Rich (Bank 2)", "Check injectors bank 2. Inspect fuel pressure. Check purge valve.", "High"),
    "P0200": ("Injector Circuit Malfunction", "Check injector wiring harness. Test injector resistance. Check PCM driver.", "High"),
    "P0217": ("Engine Overtemp Condition", "Check cooling system. Inspect radiator, water pump, thermostat. Check fan.", "Critical"),
    "P0218": ("Transmission Over Temperature", "Check trans fluid level/condition. Inspect trans cooler. Check tow load.", "Critical"),
    "P0219": ("Engine Overspeed Condition", "Check shift points. Inspect transmission. Review driving conditions.", "High"),
    "P0300": ("Random/Multiple Cylinder Misfire", "Check spark plugs, coils, fuel injectors. Check compression. Inspect wiring.", "Critical"),
    "P0301": ("Cylinder 1 Misfire", "Replace spark plug #1. Swap coil to test. Check injector #1. Compression test.", "High"),
    "P0302": ("Cylinder 2 Misfire", "Replace spark plug #2. Swap coil to test. Check injector #2. Compression test.", "High"),
    "P0303": ("Cylinder 3 Misfire", "Replace spark plug #3. Swap coil to test. Check injector #3. Compression test.", "High"),
    "P0304": ("Cylinder 4 Misfire", "Replace spark plug #4. Swap coil to test. Check injector #4. Compression test.", "High"),
    "P0305": ("Cylinder 5 Misfire", "Replace spark plug #5. Swap coil to test. Check injector #5.", "High"),
    "P0306": ("Cylinder 6 Misfire", "Replace spark plug #6. Swap coil to test. Check injector #6.", "High"),
    "P0325": ("Knock Sensor 1 Circuit (Bank 1)", "Check knock sensor wiring. Test sensor. Replace. Check for engine knock.", "Medium"),
    "P0335": ("Crankshaft Position Sensor A Circuit", "Check CKP sensor and wiring. Inspect reluctor ring. Replace sensor.", "Critical"),
    "P0336": ("Crankshaft Position Sensor Range/Performance", "Inspect CKP reluctor ring for damage. Check sensor gap. Replace.", "Critical"),
    "P0340": ("Camshaft Position Sensor Circuit (Bank 1)", "Check CMP sensor wiring. Test sensor. Inspect reluctor.", "High"),
    "P0341": ("Camshaft Position Sensor Range/Performance", "Check timing chain/belt. Inspect CMP reluctor. Replace sensor.", "High"),
    "P0400": ("EGR Flow Malfunction", "Clean EGR valve. Check EGR passages. Test EGR solenoid.", "Medium"),
    "P0401": ("EGR Flow Insufficient", "Clean EGR valve and passages. Check vacuum supply. Replace EGR.", "Medium"),
    "P0402": ("EGR Flow Excessive", "Check EGR valve for stuck open. Inspect vacuum lines. Replace.", "Medium"),
    "P0420": ("Catalyst Efficiency Below Threshold (Bank 1)", "Check for exhaust leaks. Test O2 sensors. Replace catalytic converter.", "Medium"),
    "P0421": ("Warm Up Catalyst Efficiency Below Threshold", "Check catalyst. Inspect O2 sensors. May need converter replacement.", "Medium"),
    "P0430": ("Catalyst Efficiency Below Threshold (Bank 2)", "Check exhaust leaks. Test O2 sensors. Replace cat converter bank 2.", "Medium"),
    "P0440": ("EVAP System Malfunction", "Check gas cap. Inspect EVAP hoses. Test purge and vent valves.", "Low"),
    "P0441": ("EVAP Incorrect Purge Flow", "Test purge valve. Check vacuum hoses. Inspect EVAP canister.", "Low"),
    "P0442": ("EVAP Small Leak Detected", "Check gas cap seal. Smoke test EVAP system. Check hoses.", "Low"),
    "P0443": ("EVAP Purge Control Valve Circuit", "Check purge valve wiring. Test valve. Replace if faulty.", "Low"),
    "P0446": ("EVAP Vent Control Circuit", "Check vent valve wiring. Test vent solenoid. Replace.", "Low"),
    "P0449": ("EVAP Vent Valve/Solenoid Circuit", "Check vent valve connector. Test solenoid. Replace.", "Low"),
    "P0455": ("EVAP Large Leak Detected", "Check gas cap first. Smoke test system. Check hoses and canister.", "Medium"),
    "P0456": ("EVAP Very Small Leak Detected", "Tighten gas cap. Smoke test EVAP system. Inspect all connections.", "Low"),
    "P0500": ("Vehicle Speed Sensor Malfunction", "Check VSS wiring. Test sensor. Check transmission output sensor.", "Medium"),
    "P0505": ("Idle Control System Malfunction", "Clean throttle body and IAC valve. Check for vacuum leaks.", "Medium"),
    "P0506": ("Idle Control System RPM Lower Than Expected", "Clean IAC/throttle body. Check for vacuum leaks. Reset idle learn.", "Medium"),
    "P0507": ("Idle Control System RPM Higher Than Expected", "Check for vacuum leaks. Clean throttle body. Check IAC valve.", "Medium"),
    "P0562": ("System Voltage Low", "Test battery and charging system. Check alternator. Inspect wiring.", "Medium"),
    "P0563": ("System Voltage High", "Test alternator voltage regulator. Check battery. Inspect connections.", "Medium"),
    "P0600": ("Serial Communication Link", "Check PCM connections. May indicate PCM failure. Reset and retest.", "High"),
    "P0601": ("Internal Control Module Memory Checksum Error", "Reset PCM. If persists, reprogram or replace PCM.", "Critical"),
    "P0602": ("Control Module Programming Error", "Reprogram PCM with latest calibration. Replace if needed.", "Critical"),
    "P0606": ("PCM Processor Fault", "Reset PCM. Check power and ground. Replace PCM if persists.", "Critical"),
    "P0700": ("Transmission Control System Malfunction", "Scan transmission module for specific codes. Check TCM wiring.", "High"),
    "P0706": ("Trans Range Sensor Range/Performance", "Adjust or replace TRS. Check wiring. Perform relearn.", "Medium"),
    "P0715": ("Input/Turbine Speed Sensor Circuit", "Check ISS wiring. Test sensor. Replace. Check trans fluid.", "High"),
    "P0720": ("Output Speed Sensor Circuit", "Check OSS wiring. Test sensor. Replace. Check trans fluid.", "High"),
    "P0725": ("Engine Speed Input Circuit", "Check CKP signal to TCM. Test wiring. Check TCM.", "High"),
    "P0740": ("Torque Converter Clutch Circuit", "Check TCC solenoid wiring. Test solenoid. Check trans fluid.", "High"),
    "P0741": ("Torque Converter Clutch Stuck Off", "Replace TCC solenoid. Check wiring. May need trans service.", "High"),
    "P0750": ("Shift Solenoid A Malfunction", "Check solenoid A wiring. Test solenoid resistance. Replace.", "High"),
    "P0755": ("Shift Solenoid B Malfunction", "Check solenoid B wiring. Test solenoid resistance. Replace.", "High"),
    "B0001": ("Driver Frontal Stage 1 Deployment Control", "Check airbag system. Inspect clockspring. Scan airbag module.", "Critical"),
    "B0100": ("Electronic Frontal Sensor 1", "Check frontal crash sensor wiring. Replace sensor.", "Critical"),
    "C0035": ("Left Front Wheel Speed Circuit", "Check LF wheel speed sensor and wiring. Inspect tone ring.", "Medium"),
    "C0040": ("Right Front Wheel Speed Circuit", "Check RF wheel speed sensor and wiring. Inspect tone ring.", "Medium"),
    "C0045": ("Left Rear Wheel Speed Circuit", "Check LR wheel speed sensor and wiring. Inspect tone ring.", "Medium"),
    "C0050": ("Right Rear Wheel Speed Circuit", "Check RR wheel speed sensor and wiring. Inspect tone ring.", "Medium"),
    "C0060": ("Left Front ABS Solenoid 1 Circuit", "Check ABS module wiring. Test solenoid. May need ABS module.", "High"),
    "U0001": ("High Speed CAN Communication Bus", "Check CAN bus wiring. Look for shorts. Check terminating resistors.", "Critical"),
    "U0073": ("Control Module Communication Bus Off", "Check CAN bus wiring. Inspect connectors. Test bus voltage.", "Critical"),
    "U0100": ("Lost Communication with ECM/PCM", "Check ECM power and ground. Inspect CAN wiring. Test ECM.", "Critical"),
    "U0101": ("Lost Communication with TCM", "Check TCM power/ground. Inspect CAN wiring to TCM.", "High"),
    "U0121": ("Lost Communication with ABS", "Check ABS module power/ground. Inspect CAN wiring.", "High"),
    "U0140": ("Lost Communication with BCM", "Check BCM power/ground. Inspect CAN wiring to BCM.", "High"),
    "U0155": ("Lost Communication with Instrument Cluster", "Check cluster connections. Inspect CAN wiring.", "Medium"),
    "U0164": ("Lost Communication with HVAC", "Check HVAC module connections. Inspect CAN bus.", "Low"),
    "U0401": ("Invalid Data Received from ECM/PCM", "Check CAN bus integrity. Reset modules. Check for interference.", "High"),
    "U0402": ("Invalid Data Received from TCM", "Check CAN wiring. Reset TCM. Check for bus errors.", "High"),
}

# ─── Serial Communication ─────────────────────────────────

def open_serial():
    global serial_fd, state
    try:
        os.system(f"stty -F {SERIAL_PORT} {BAUD_RATE} raw -echo 2>/dev/null")
        serial_fd = os.open(SERIAL_PORT, os.O_RDWR)
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

def serial_read(size=4096, timeout=2.0):
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
        time.sleep(0.05)
    return bytes(data)

def log(msg):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    entry = f"[{ts}] {msg}"
    state["scan_log"].append(entry)
    if len(state["scan_log"]) > 500:
        state["scan_log"] = state["scan_log"][-500:]

# ─── Frame Parsing ─────────────────────────────────────────

def parse_frames(data):
    """Parse CRLF-delimited MCU frames"""
    frames = []
    i = 0
    while i < len(data) - 1:
        if data[i] == 0x0D and data[i+1] == 0x0A and i + 2 < len(data):
            start = i + 2
            end = len(data)
            for j in range(start, len(data) - 1):
                if data[j] == 0x0D and data[j+1] == 0x0A:
                    end = j
                    break
            if end > start:
                frames.append(bytes(data[start:end]))
            i = end
        else:
            i += 1
    return frames

def store_sensor_frame(frame):
    """Store raw 0x08 sensor frame for display"""
    if len(frame) < 9 or frame[0] != 0x08:
        return
    state["frame_counts"]["sensor"] += 1
    # Store last 20 raw payloads for the hex monitor
    hex_str = " ".join(f"{b:02X}" for b in frame)
    state["mcu_raw_bytes"].append({"hex": hex_str, "t": time.time()})
    if len(state["mcu_raw_bytes"]) > 20:
        state["mcu_raw_bytes"] = state["mcu_raw_bytes"][-20:]

def process_heartbeat(frame):
    """Process type 0x04 heartbeat frame"""
    if len(frame) >= 5 and frame[0] == 0x04 and frame[1] == 0x71:
        status = frame[4]
        state["mil"] = bool((status >> 7) & 1)
        state["dtc_count"] = status & 0x7F
        state["last_heartbeat"] = datetime.now().strftime("%H:%M:%S")
        return True
    return False

# ─── Background Reader ─────────────────────────────────────

def background_reader():
    """Continuously read and parse serial data"""
    global running
    fps_counter = 0
    fps_time = time.time()
    while running:
        if not state["connected"]:
            time.sleep(1)
            continue
        try:
            data = serial_read(4096, 0.5)
            if data:
                frames = parse_frames(data)
                for frame in frames:
                    hex_str = " ".join(f"{b:02X}" for b in frame)
                    ftype = frame[0] if frame else 0

                    if ftype == 0x04:
                        process_heartbeat(frame)
                        state["frame_counts"]["heartbeat"] += 1
                        state["raw_frames"].append({"type": "heartbeat", "hex": hex_str, "t": time.time()})
                    elif ftype == 0x08:
                        store_sensor_frame(frame)
                        state["raw_frames"].append({"type": "sensor", "hex": hex_str, "t": time.time()})
                    elif ftype == 0x11:
                        state["raw_frames"].append({"type": "extended", "hex": hex_str, "t": time.time()})
                        log(f"EXTENDED FRAME: {hex_str}")
                    elif ftype == 0x03:
                        state["raw_frames"].append({"type": "status", "hex": hex_str, "t": time.time()})
                    elif ftype == 0x1D:
                        state["raw_frames"].append({"type": "ident", "hex": hex_str, "t": time.time()})
                        log(f"IDENT FRAME: {hex_str}")
                    else:
                        state["raw_frames"].append({"type": f"0x{ftype:02X}", "hex": hex_str, "t": time.time()})

                if len(state["raw_frames"]) > 200:
                    state["raw_frames"] = state["raw_frames"][-200:]

                fps_counter += len(frames)
                now = time.time()
                if now - fps_time >= 1.0:
                    state["frames_per_sec"] = fps_counter
                    fps_counter = 0
                    fps_time = now

        except Exception as e:
            log(f"Reader error: {e}")
            time.sleep(0.5)

        time.sleep(0.1)

# ─── Commands ──────────────────────────────────────────────

def cmd_scan_dtcs():
    """Send MCU commands to try reading DTCs"""
    log("Starting DTC scan...")
    commands = [
        ("Status 0x50", bytes([0x55, 0x04, 0x50, 0x00, 0x00, 0x01, 0x51])),
        ("Warning 0x60/0", bytes([0x55, 0x04, 0x60, 0x00, 0x00, 0x01, 0x61])),
        ("Warning 0x60/1", bytes([0x55, 0x04, 0x60, 0x01, 0x00, 0x01, 0x60])),
        ("Warning 0x60/2", bytes([0x55, 0x04, 0x60, 0x02, 0x00, 0x01, 0x63])),
        ("Diag 0x70/0", bytes([0x55, 0x04, 0x70, 0x00, 0x00, 0x01, 0x71])),
        ("Diag 0x70/1", bytes([0x55, 0x04, 0x70, 0x01, 0x00, 0x01, 0x70])),
        ("Diag 0x70/2", bytes([0x55, 0x04, 0x70, 0x02, 0x00, 0x01, 0x73])),
        ("VIN 0x80", bytes([0x55, 0x04, 0x80, 0x00, 0x00, 0x01, 0x81])),
    ]

    dtcs_found = []

    for name, cmd in commands:
        log(f"Sending {name}: {cmd.hex()}")
        serial_write(cmd)
        time.sleep(1.5)
        data = serial_read(4096, 1.0)
        if data:
            frames = parse_frames(data)
            for frame in frames:
                if frame[0] == 0x04:
                    process_heartbeat(frame)
                elif frame[0] == 0x11 and len(frame) >= 10:
                    hex_str = " ".join(f"{b:02X}" for b in frame)
                    log(f"  RESPONSE: {hex_str}")
                    # Try to extract DTC bytes
                    for k in range(7, len(frame) - 3, 2):
                        b1, b2 = frame[k], frame[k+1]
                        if b1 != 0 or b2 != 0:
                            dtc_type = ["P", "C", "B", "U"][(b1 >> 6) & 3]
                            dtc_num = ((b1 & 0x3F) << 8) | b2
                            code = f"{dtc_type}{dtc_num:04d}"
                            if code not in dtcs_found:
                                dtcs_found.append(code)
                                log(f"  Found DTC: {code}")

    if dtcs_found:
        state["dtcs"] = dtcs_found
        log(f"Scan complete: {len(dtcs_found)} codes found")
    else:
        log(f"MCU confirms {state['dtc_count']} DTCs but individual codes require CAN bus service handshake")
        log("The MCU will not enumerate codes without proper initialization from zxwCanbusService")

    return dtcs_found

def cmd_clear_dtcs():
    """Send clear DTCs command"""
    log("Sending Clear DTCs (Mode 04)...")
    commands = [
        bytes([0x55, 0x02, 0x04, 0x00, 0x04]),
        bytes([0x55, 0x01, 0x04, 0x04]),
    ]
    for cmd in commands:
        serial_write(cmd)
        time.sleep(1)
    log("Clear command sent. Re-scan to verify.")
    return True

def cmd_send_raw(hex_str):
    """Send raw hex command to MCU"""
    try:
        data = bytes.fromhex(hex_str.replace(" ", ""))
        log(f"Sending raw: {data.hex()}")
        serial_write(data)
        time.sleep(1.5)
        resp = serial_read(4096, 1.0)
        if resp:
            frames = parse_frames(resp)
            results = []
            for f in frames:
                results.append(" ".join(f"{b:02X}" for b in f))
            return results
        return []
    except Exception as e:
        log(f"Raw send error: {e}")
        return []

# ─── HTML UI ───────────────────────────────────────────────

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>OBD2 Pro Tuner</title>
<style>
:root {
    --bg: #0a0a0f;
    --panel: #12121a;
    --border: #1e1e2e;
    --accent: #00ff88;
    --accent2: #00ccff;
    --red: #ff3355;
    --orange: #ff8800;
    --yellow: #ffcc00;
    --text: #e0e0e0;
    --dim: #666680;
}
* { margin:0; padding:0; box-sizing:border-box; }
body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Courier New', monospace;
    overflow-x: hidden;
    min-height: 100vh;
}
.header {
    background: linear-gradient(135deg, #0d0d15 0%, #151525 100%);
    border-bottom: 2px solid var(--accent);
    padding: 10px 16px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    position: sticky;
    top: 0;
    z-index: 100;
}
.header h1 {
    font-size: 18px;
    color: var(--accent);
    text-shadow: 0 0 20px rgba(0,255,136,0.3);
}
.header .status {
    display: flex;
    gap: 12px;
    align-items: center;
    font-size: 12px;
}
.status-dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    display: inline-block;
    animation: pulse 2s infinite;
}
.status-dot.on { background: var(--accent); box-shadow: 0 0 8px var(--accent); }
.status-dot.off { background: var(--red); }
.status-dot.warn { background: var(--orange); animation: pulse-warn 1s infinite; }
@keyframes pulse { 0%,100% { opacity:1; } 50% { opacity:0.5; } }
@keyframes pulse-warn { 0%,100% { opacity:1; } 50% { opacity:0.3; } }

.mil-badge {
    padding: 3px 10px;
    border-radius: 4px;
    font-weight: bold;
    font-size: 11px;
    letter-spacing: 1px;
}
.mil-on { background: var(--red); color: white; animation: pulse-warn 1s infinite; }
.mil-off { background: #1a3a1a; color: var(--accent); }

.tabs {
    display: flex;
    background: var(--panel);
    border-bottom: 1px solid var(--border);
    overflow-x: auto;
    -webkit-overflow-scrolling: touch;
}
.tab {
    padding: 10px 16px;
    font-size: 12px;
    color: var(--dim);
    cursor: pointer;
    white-space: nowrap;
    border-bottom: 2px solid transparent;
    transition: all 0.2s;
    font-family: 'Courier New', monospace;
    background: none;
    border-top: none;
    border-left: none;
    border-right: none;
}
.tab:hover { color: var(--text); }
.tab.active { color: var(--accent); border-bottom-color: var(--accent); }

.page { display: none; padding: 12px; }
.page.active { display: block; }

/* ─── Gauges ─── */
.gauge-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 10px;
    margin-bottom: 12px;
}
.gauge {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 12px;
    text-align: center;
    position: relative;
    overflow: hidden;
}
.gauge::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 3px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
    opacity: 0.5;
}
.gauge .label { font-size: 10px; color: var(--dim); text-transform: uppercase; letter-spacing: 1px; }
.gauge .value { font-size: 28px; font-weight: bold; color: var(--accent); margin: 4px 0; }
.gauge .unit { font-size: 11px; color: var(--dim); }
.gauge.warn .value { color: var(--orange); }
.gauge.crit .value { color: var(--red); }

.gauge-bar {
    width: 100%;
    height: 4px;
    background: var(--border);
    border-radius: 2px;
    margin-top: 6px;
    overflow: hidden;
}
.gauge-bar-fill {
    height: 100%;
    border-radius: 2px;
    transition: width 0.3s;
    background: var(--accent);
}

/* ─── DTC Panel ─── */
.dtc-card {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 12px;
    margin-bottom: 8px;
    border-left: 4px solid var(--accent);
}
.dtc-card.critical { border-left-color: var(--red); }
.dtc-card.high { border-left-color: var(--orange); }
.dtc-card.medium { border-left-color: var(--yellow); }
.dtc-card.low { border-left-color: var(--accent2); }

.dtc-code {
    font-size: 20px;
    font-weight: bold;
    color: var(--accent);
    margin-bottom: 4px;
}
.dtc-card.critical .dtc-code { color: var(--red); }
.dtc-card.high .dtc-code { color: var(--orange); }

.dtc-desc { font-size: 13px; color: var(--text); margin-bottom: 6px; }
.dtc-fix { font-size: 11px; color: var(--accent2); line-height: 1.4; }
.dtc-severity {
    display: inline-block;
    padding: 2px 8px;
    border-radius: 3px;
    font-size: 10px;
    font-weight: bold;
    margin-bottom: 6px;
}
.sev-critical { background: rgba(255,51,85,0.2); color: var(--red); }
.sev-high { background: rgba(255,136,0,0.2); color: var(--orange); }
.sev-medium { background: rgba(255,204,0,0.2); color: var(--yellow); }
.sev-low { background: rgba(0,204,255,0.2); color: var(--accent2); }

/* ─── Buttons ─── */
.btn {
    padding: 12px 20px;
    border: none;
    border-radius: 6px;
    font-family: 'Courier New', monospace;
    font-size: 14px;
    font-weight: bold;
    cursor: pointer;
    text-transform: uppercase;
    letter-spacing: 1px;
    transition: all 0.2s;
    width: 100%;
    margin-bottom: 8px;
}
.btn:active { transform: scale(0.98); }
.btn-green { background: var(--accent); color: #000; }
.btn-green:hover { box-shadow: 0 0 20px rgba(0,255,136,0.3); }
.btn-red { background: var(--red); color: white; }
.btn-blue { background: var(--accent2); color: #000; }
.btn-orange { background: var(--orange); color: #000; }
.btn-dark { background: var(--panel); color: var(--text); border: 1px solid var(--border); }

.btn-row { display: flex; gap: 8px; margin-bottom: 12px; }
.btn-row .btn { flex: 1; }

/* ─── Log/Raw ─── */
.log-box {
    background: #050508;
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 10px;
    font-size: 11px;
    line-height: 1.5;
    max-height: 400px;
    overflow-y: auto;
    white-space: pre-wrap;
    word-break: break-all;
}
.log-box .ts { color: var(--dim); }
.log-box .info { color: var(--accent2); }
.log-box .warn { color: var(--orange); }
.log-box .err { color: var(--red); }
.log-box .ok { color: var(--accent); }

.raw-frame {
    font-size: 11px;
    padding: 3px 6px;
    margin: 2px 0;
    border-radius: 3px;
    display: inline-block;
}
.raw-frame.heartbeat { background: rgba(0,255,136,0.1); color: var(--accent); }
.raw-frame.sensor { background: rgba(0,204,255,0.05); color: var(--dim); }
.raw-frame.extended { background: rgba(255,204,0,0.15); color: var(--yellow); }
.raw-frame.status { background: rgba(255,136,0,0.1); color: var(--orange); }

/* ─── Raw Command ─── */
.cmd-input {
    width: 100%;
    padding: 10px;
    background: #050508;
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--accent);
    font-family: 'Courier New', monospace;
    font-size: 14px;
    margin-bottom: 8px;
}
.cmd-input:focus { outline: none; border-color: var(--accent); }

/* ─── Info Panel ─── */
.info-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
}
.info-item {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 10px;
}
.info-item .label { font-size: 10px; color: var(--dim); text-transform: uppercase; }
.info-item .value { font-size: 13px; color: var(--accent); margin-top: 2px; }

.section-title {
    font-size: 14px;
    color: var(--accent);
    margin: 16px 0 8px 0;
    padding-bottom: 4px;
    border-bottom: 1px solid var(--border);
}

/* ─── Canvas for graph ─── */
#liveGraph {
    width: 100%;
    height: 200px;
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
}

.spinner {
    display: inline-block;
    width: 16px;
    height: 16px;
    border: 2px solid var(--dim);
    border-top-color: var(--accent);
    border-radius: 50%;
    animation: spin 0.8s linear infinite;
    margin-right: 8px;
    vertical-align: middle;
}
@keyframes spin { to { transform: rotate(360deg); } }

.scanning-overlay {
    display: none;
    position: fixed;
    top: 0; left: 0; right: 0; bottom: 0;
    background: rgba(0,0,0,0.8);
    z-index: 200;
    justify-content: center;
    align-items: center;
    flex-direction: column;
}
.scanning-overlay.active { display: flex; }
.scanning-text {
    color: var(--accent);
    font-size: 18px;
    margin-top: 20px;
}
</style>
</head>
<body>

<div class="header">
    <h1>OBD2 PRO TUNER</h1>
    <div class="status">
        <span class="status-dot" id="connDot"></span>
        <span id="connText">---</span>
        <span class="mil-badge" id="milBadge">MIL</span>
    </div>
</div>

<div class="tabs">
    <button class="tab active" onclick="showPage('dashboard')">DASH</button>
    <button class="tab" onclick="showPage('dtcs')">DTCs</button>
    <button class="tab" onclick="showPage('live')">LIVE</button>
    <button class="tab" onclick="showPage('raw')">RAW</button>
    <button class="tab" onclick="showPage('tools')">TOOLS</button>
    <button class="tab" onclick="showPage('info')">INFO</button>
</div>

<!-- ═══ DASHBOARD ═══ -->
<div class="page active" id="page-dashboard">
    <div style="background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:12px;margin-bottom:12px">
        <div style="font-size:14px;color:var(--accent);margin-bottom:8px">2008 TOYOTA FJ CRUISER 4WD</div>
        <div class="gauge-grid" style="grid-template-columns: repeat(2,1fr)">
            <div class="gauge" id="g-mil">
                <div class="label">CHECK ENGINE</div>
                <div class="value" id="v-mil" style="font-size:22px">---</div>
                <div class="unit">malfunction indicator</div>
            </div>
            <div class="gauge" id="g-dtccount">
                <div class="label">FAULT CODES</div>
                <div class="value" id="v-dtccount">---</div>
                <div class="unit">DTCs stored</div>
            </div>
            <div class="gauge">
                <div class="label">MCU LINK</div>
                <div class="value" id="v-link" style="font-size:18px">---</div>
                <div class="unit">serial connection</div>
            </div>
            <div class="gauge">
                <div class="label">DATA RATE</div>
                <div class="value" id="v-fps" style="font-size:18px">---</div>
                <div class="unit">frames/sec</div>
            </div>
        </div>
    </div>

    <div class="btn-row">
        <button class="btn btn-green" onclick="doScan()">SCAN DTCs</button>
        <button class="btn btn-red" onclick="doClear()">CLEAR CODES</button>
    </div>

    <div style="background:rgba(255,204,0,0.08);border:1px solid rgba(255,204,0,0.2);border-radius:8px;padding:12px;margin-bottom:12px">
        <div style="font-size:12px;color:var(--yellow);margin-bottom:6px">CAN BUS SERVICE STATUS</div>
        <div style="font-size:11px;color:var(--dim);line-height:1.5" id="canbusStatus">
            The LuZhengCanParseToyotaFJ decoder needs the zxwCanbusService running
            to decode live RPM, Speed, Coolant, and Throttle data from your FJ Cruiser's CAN bus.
            <br><br>
            <b style="color:var(--accent2)">To activate:</b> Open the CAN Bus app from the head unit home screen
            and select Toyota > FJ Cruiser. This starts the decoder service.
        </div>
    </div>

    <div class="section-title">LIVE MCU STREAM</div>
    <div class="log-box" id="dashLog" style="max-height:150px;font-size:10px"></div>

    <div class="section-title">SCAN LOG</div>
    <div class="log-box" id="scanLog" style="max-height:150px"></div>
</div>

<!-- ═══ DTCs ═══ -->
<div class="page" id="page-dtcs">
    <div class="btn-row">
        <button class="btn btn-green" onclick="doScan()">SCAN FOR CODES</button>
        <button class="btn btn-red" onclick="doClear()">CLEAR ALL CODES</button>
    </div>
    <div id="dtcSummary" style="margin-bottom:12px; font-size:13px; color:var(--dim)"></div>
    <div id="dtcList"></div>
</div>

<!-- ═══ LIVE DATA ═══ -->
<div class="page" id="page-live">
    <div class="section-title">MCU RAW DATA STREAM</div>
    <div style="font-size:11px;color:var(--dim);margin-bottom:8px">
        Live 0x08 sensor frames from Szchoiceway MCU. These are raw CAN bus relay frames —
        decoded vehicle parameters (RPM, Speed, etc.) require the CAN bus service + Toyota FJ profile.
    </div>
    <div class="log-box" id="liveHex" style="max-height:300px;font-size:11px"></div>

    <div class="section-title" style="margin-top:16px">FRAME STATISTICS</div>
    <div class="gauge-grid" style="grid-template-columns: repeat(3,1fr)">
        <div class="gauge">
            <div class="label">SENSOR (0x08)</div>
            <div class="value" id="v-fc-sensor" style="font-size:20px">0</div>
        </div>
        <div class="gauge">
            <div class="label">HEARTBEAT (0x04)</div>
            <div class="value" id="v-fc-hb" style="font-size:20px">0</div>
        </div>
        <div class="gauge">
            <div class="label">EXTENDED (0x11)</div>
            <div class="value" id="v-fc-ext" style="font-size:20px">0</div>
        </div>
    </div>

    <div class="section-title" style="margin-top:12px">HEARTBEAT DECODE</div>
    <div style="background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:12px;font-size:12px">
        <div>Frame: <span style="color:var(--accent)" id="hb-hex">04 71 01 00 ??</span></div>
        <div style="margin-top:4px">Byte 4 = <span id="hb-byte4" style="color:var(--yellow)">0x??</span> = <span id="hb-binary" style="color:var(--accent2)">????????</span></div>
        <div style="margin-top:4px">Bit 7 = MIL: <span id="hb-mil" style="font-weight:bold">?</span></div>
        <div>Bits 0-6 = DTCs: <span id="hb-dtc" style="color:var(--accent);font-weight:bold">?</span></div>
        <div style="margin-top:4px;color:var(--dim)">Last heartbeat: <span id="hb-time">---</span></div>
    </div>
</div>

<!-- ═══ RAW DATA ═══ -->
<div class="page" id="page-raw">
    <div class="section-title">SEND RAW COMMAND</div>
    <input class="cmd-input" id="rawCmd" placeholder="Hex bytes e.g. 55 04 60 00 00 01 61" />
    <div class="btn-row">
        <button class="btn btn-blue" onclick="sendRaw()">SEND</button>
        <button class="btn btn-dark" onclick="clearLog()">CLEAR LOG</button>
    </div>

    <div class="section-title">QUICK COMMANDS</div>
    <div style="display:flex;flex-wrap:wrap;gap:6px;margin-bottom:12px">
        <button class="btn btn-dark" style="width:auto;font-size:11px;padding:6px 10px" onclick="quickCmd('55 04 50 00 00 01 51')">Status 0x50</button>
        <button class="btn btn-dark" style="width:auto;font-size:11px;padding:6px 10px" onclick="quickCmd('55 04 60 00 00 01 61')">Warning 0x60</button>
        <button class="btn btn-dark" style="width:auto;font-size:11px;padding:6px 10px" onclick="quickCmd('55 04 70 00 00 01 71')">Diag 0x70</button>
        <button class="btn btn-dark" style="width:auto;font-size:11px;padding:6px 10px" onclick="quickCmd('55 04 80 00 00 01 81')">VIN 0x80</button>
        <button class="btn btn-dark" style="width:auto;font-size:11px;padding:6px 10px" onclick="quickCmd('55 02 04 00 04')">Clear DTCs</button>
        <button class="btn btn-dark" style="width:auto;font-size:11px;padding:6px 10px" onclick="quickCmd('2E 04 60 00 00 01 8D')">2E Warning</button>
        <button class="btn btn-dark" style="width:auto;font-size:11px;padding:6px 10px" onclick="quickCmd('AA 55 04 60 00 00 01 61')">AA55 Warning</button>
        <button class="btn btn-dark" style="width:auto;font-size:11px;padding:6px 10px" onclick="quickCmd('FF 55 04 70 00 00 01 71')">FF55 Diag</button>
    </div>

    <div class="section-title">RAW FRAMES</div>
    <div class="log-box" id="rawLog" style="max-height:300px"></div>
</div>

<!-- ═══ TOOLS ═══ -->
<div class="page" id="page-tools">
    <div class="section-title">DIAGNOSTICS</div>
    <button class="btn btn-green" onclick="doScan()">FULL DTC SCAN</button>
    <button class="btn btn-red" onclick="doClear()">CLEAR ALL DTCs</button>
    <button class="btn btn-blue" onclick="doConnect()">RECONNECT SERIAL</button>

    <div class="section-title">EXPORT</div>
    <button class="btn btn-orange" onclick="doExport()">SAVE REPORT TO DOWNLOADS</button>

    <div class="section-title">MCU PROTOCOL</div>
    <div style="font-size:12px;color:var(--dim);line-height:1.6;background:var(--panel);padding:12px;border-radius:6px;border:1px solid var(--border)">
        <b style="color:var(--accent)">Szchoiceway MCU Protocol</b><br>
        Port: /dev/ttyHS1 @ 115200 baud<br>
        Framing: CRLF-delimited binary<br><br>
        <b style="color:var(--accent)">Frame Types:</b><br>
        0x03 = Status (4B)<br>
        0x04 = Heartbeat (5B) — MIL + DTC count<br>
        0x08 = Sensor/CAN data (9B)<br>
        0x11 = Extended response (variable)<br>
        0x1D = Identification (30+B)<br><br>
        <b style="color:var(--accent)">Heartbeat: 04 71 01 00 XX</b><br>
        Bit 7 = MIL, Bits 0-6 = DTC count<br><br>
        <b style="color:var(--accent)">Command: 55 LEN CMD SUB DATA CHK</b><br>
        0x50=Status, 0x60=Warning, 0x70=Diag, 0x80=VIN
    </div>
</div>

<!-- ═══ INFO ═══ -->
<div class="page" id="page-info">
    <div class="section-title">VEHICLE CONNECTION</div>
    <div class="info-grid">
        <div class="info-item"><div class="label">Vehicle</div><div class="value" id="i-veh">---</div></div>
        <div class="info-item"><div class="label">CAN Profile</div><div class="value" id="i-canp">---</div></div>
        <div class="info-item"><div class="label">Head Unit</div><div class="value" id="i-hu">---</div></div>
        <div class="info-item"><div class="label">Firmware</div><div class="value" id="i-fw">---</div></div>
        <div class="info-item"><div class="label">MCU ID</div><div class="value" id="i-mcu">---</div></div>
        <div class="info-item"><div class="label">Platform</div><div class="value" id="i-plat">---</div></div>
        <div class="info-item"><div class="label">Serial Port</div><div class="value" id="i-port">---</div></div>
        <div class="info-item"><div class="label">Baud Rate</div><div class="value" id="i-baud">---</div></div>
        <div class="info-item"><div class="label">Protocol</div><div class="value" id="i-proto">---</div></div>
        <div class="info-item"><div class="label">CAN Bus Svc</div><div class="value" id="i-cansvc">---</div></div>
        <div class="info-item"><div class="label">Last Heartbeat</div><div class="value" id="i-hb">---</div></div>
        <div class="info-item"><div class="label">Status</div><div class="value" id="i-status">---</div></div>
    </div>

    <div class="section-title" style="margin-top:16px">ABOUT</div>
    <div style="font-size:12px;color:var(--dim);line-height:1.6;background:var(--panel);padding:12px;border-radius:6px;border:1px solid var(--border)">
        <b style="color:var(--accent)">MobileCLI OBD2 Pro Tuner v1.0</b><br>
        Built with MobileCLI + Claude AI<br>
        Direct serial MCU communication<br>
        No Bluetooth adapter required<br><br>
        <b style="color:var(--accent2)">github.com/MobileDevCLI</b>
    </div>
</div>

<div class="scanning-overlay" id="scanOverlay">
    <div class="spinner" style="width:40px;height:40px;border-width:3px"></div>
    <div class="scanning-text" id="scanText">Scanning...</div>
</div>

<script>
let pollInterval;
let graphCtx;
let currentPage = 'dashboard';

function showPage(name) {
    document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
    document.getElementById('page-'+name).classList.add('active');
    document.querySelectorAll('.tab')[['dashboard','dtcs','live','raw','tools','info'].indexOf(name)].classList.add('active');
    currentPage = name;
    if (name === 'live') initGraph();
}

// ─── API calls ───
async function api(endpoint, body) {
    try {
        const r = await fetch('/api/' + endpoint, {
            method: 'POST',
            headers: {'Content-Type':'application/json'},
            body: JSON.stringify(body || {})
        });
        return await r.json();
    } catch(e) {
        console.error(e);
        return {error: e.message};
    }
}

async function poll() {
    const d = await api('state');
    if (d.error) return;
    updateDashboard(d);
}

function updateDashboard(d) {
    // Connection
    const dot = document.getElementById('connDot');
    const txt = document.getElementById('connText');
    if (d.connected) {
        dot.className = 'status-dot on';
        txt.textContent = 'CONNECTED';
    } else {
        dot.className = 'status-dot off';
        txt.textContent = 'DISCONNECTED';
    }

    // MIL badge
    const mil = document.getElementById('milBadge');
    if (d.mil) {
        mil.className = 'mil-badge mil-on';
        mil.textContent = 'MIL ON';
    } else {
        mil.className = 'mil-badge mil-off';
        mil.textContent = 'MIL OFF';
    }

    // Dashboard gauges
    const milG = document.getElementById('g-mil');
    const milV = document.getElementById('v-mil');
    if (d.mil) { milV.textContent = 'ON'; milV.style.color='var(--red)'; milG.className='gauge crit'; }
    else { milV.textContent = 'OFF'; milV.style.color='var(--accent)'; milG.className='gauge'; }

    const dtcG = document.getElementById('g-dtccount');
    document.getElementById('v-dtccount').textContent = d.dtc_count;
    dtcG.className = 'gauge' + (d.dtc_count > 0 ? ' crit' : '');

    document.getElementById('v-link').textContent = d.connected ? 'ACTIVE' : 'NONE';
    document.getElementById('v-link').style.color = d.connected ? 'var(--accent)' : 'var(--red)';
    document.getElementById('v-fps').textContent = d.frames_per_sec || 0;

    // MCU stream on dashboard
    if (currentPage === 'dashboard') {
        const dashLog = document.getElementById('dashLog');
        if (d.mcu_raw_bytes && d.mcu_raw_bytes.length > 0) {
            dashLog.innerHTML = d.mcu_raw_bytes.slice(-10).map(f =>
                '<span style="color:var(--accent2)">' + f.hex + '</span>'
            ).join('\\n');
            dashLog.scrollTop = dashLog.scrollHeight;
        }

        const scanLog = document.getElementById('scanLog');
        scanLog.innerHTML = d.scan_log.slice(-20).map(l => {
            let cls = 'info';
            if (l.includes('Error') || l.includes('failed')) cls = 'err';
            else if (l.includes('Found') || l.includes('complete')) cls = 'ok';
            return '<span class="'+cls+'">'+escHtml(l)+'</span>';
        }).join('\\n');
        scanLog.scrollTop = scanLog.scrollHeight;
    }

    // Live data page
    if (currentPage === 'live') {
        const liveHex = document.getElementById('liveHex');
        if (d.mcu_raw_bytes) {
            liveHex.innerHTML = d.mcu_raw_bytes.map(f =>
                '<span style="color:var(--accent2)">' + f.hex + '</span>'
            ).join('\\n');
            liveHex.scrollTop = liveHex.scrollHeight;
        }
        document.getElementById('v-fc-sensor').textContent = d.frame_counts.sensor || 0;
        document.getElementById('v-fc-hb').textContent = d.frame_counts.heartbeat || 0;
        document.getElementById('v-fc-ext').textContent = d.frame_counts.extended || 0;

        // Heartbeat decode
        if (d.dtc_count >= 0) {
            const statusByte = (d.mil ? 0x80 : 0) | (d.dtc_count & 0x7F);
            document.getElementById('hb-hex').textContent = '04 71 01 00 ' + statusByte.toString(16).toUpperCase().padStart(2,'0');
            document.getElementById('hb-byte4').textContent = '0x' + statusByte.toString(16).toUpperCase().padStart(2,'0');
            document.getElementById('hb-binary').textContent = statusByte.toString(2).padStart(8,'0');
            document.getElementById('hb-mil').textContent = d.mil ? 'ON' : 'OFF';
            document.getElementById('hb-mil').style.color = d.mil ? 'var(--red)' : 'var(--accent)';
            document.getElementById('hb-dtc').textContent = d.dtc_count;
            document.getElementById('hb-time').textContent = d.last_heartbeat || '---';
        }
    }

    // Raw frames
    if (currentPage === 'raw') {
        const rawEl = document.getElementById('rawLog');
        rawEl.innerHTML = d.raw_frames.slice(-50).map(f =>
            '<span class="raw-frame '+f.type+'">'+f.type.toUpperCase()+': '+f.hex+'</span>'
        ).join('\\n');
        rawEl.scrollTop = rawEl.scrollHeight;
    }

    // DTC list
    if (currentPage === 'dtcs') updateDTCPage(d);

    // Info
    if (currentPage === 'info') {
        document.getElementById('i-veh').textContent = d.vehicle || '---';
        document.getElementById('i-canp').textContent = d.canbus_profile || '---';
        document.getElementById('i-hu').textContent = d.head_unit;
        document.getElementById('i-fw').textContent = d.firmware;
        document.getElementById('i-mcu').textContent = d.mcu_id;
        document.getElementById('i-plat').textContent = d.platform;
        document.getElementById('i-port').textContent = d.port;
        document.getElementById('i-baud').textContent = d.baud;
        document.getElementById('i-proto').textContent = d.protocol;
        document.getElementById('i-cansvc').textContent = d.canbus_service || 'Unknown';
        document.getElementById('i-hb').textContent = d.last_heartbeat || 'Waiting...';
        document.getElementById('i-status').textContent = d.connected ? 'Online' : 'Offline';
    }
}

function setGauge(id, val, max, suffix, cls) {}

function updateDTCPage(d) {
    const summary = document.getElementById('dtcSummary');
    const list = document.getElementById('dtcList');

    summary.innerHTML = d.mil
        ? '<span style="color:var(--red)">CHECK ENGINE LIGHT: ON</span> &mdash; ' + d.dtc_count + ' codes stored'
        : '<span style="color:var(--accent)">No warning lights active</span>';

    if (d.dtcs && d.dtcs.length > 0) {
        list.innerHTML = d.dtcs.map((dtc, i) => {
            const info = d.dtc_info[dtc] || {desc:'Unknown code', fix:'Consult vehicle service manual', sev:'Medium'};
            const sevCls = info.sev.toLowerCase();
            return '<div class="dtc-card '+sevCls+'">' +
                '<div class="dtc-code">#'+(i+1)+' '+dtc+'</div>' +
                '<span class="dtc-severity sev-'+sevCls+'">'+info.sev.toUpperCase()+'</span>' +
                '<div class="dtc-desc">'+escHtml(info.desc)+'</div>' +
                '<div class="dtc-fix">FIX: '+escHtml(info.fix)+'</div>' +
                '</div>';
        }).join('');
    } else if (d.dtc_count > 0) {
        list.innerHTML = '<div class="dtc-card">' +
            '<div class="dtc-code">'+d.dtc_count+' DTCs Stored</div>' +
            '<div class="dtc-desc">MCU confirms codes exist but requires CAN bus service for enumeration.</div>' +
            '<div class="dtc-fix">Tap SCAN FOR CODES to attempt extraction, or use the head unit CAN Bus app.</div></div>';
    } else {
        list.innerHTML = '<div style="text-align:center;padding:40px;color:var(--accent)">No DTCs stored. Vehicle is healthy!</div>';
    }
}

// ─── Graph ───
function initGraph() {
    const c = document.getElementById('liveGraph');
    c.width = c.offsetWidth * 2;
    c.height = 400;
    graphCtx = c.getContext('2d');
}

function drawGraph(history) {
    if (!graphCtx) return;
    const c = graphCtx.canvas;
    const w = c.width, h = c.height;
    graphCtx.clearRect(0, 0, w, h);

    // Grid
    graphCtx.strokeStyle = '#1e1e2e';
    graphCtx.lineWidth = 1;
    for (let y = 0; y < h; y += h/5) {
        graphCtx.beginPath(); graphCtx.moveTo(0,y); graphCtx.lineTo(w,y); graphCtx.stroke();
    }

    const last = history.slice(-100);
    if (last.length < 2) return;

    const series = [
        {key:'rpm', max:8000, color:'#00ff88', chk:'chk-rpm'},
        {key:'speed', max:250, color:'#00ccff', chk:'chk-speed'},
        {key:'coolant', max:130, color:'#ff8800', chk:'chk-coolant'},
        {key:'throttle', max:100, color:'#ffcc00', chk:'chk-throttle'},
        {key:'voltage', max:16, color:'#ff3355', chk:'chk-voltage'},
    ];

    series.forEach(s => {
        if (!document.getElementById(s.chk).checked) return;
        graphCtx.strokeStyle = s.color;
        graphCtx.lineWidth = 3;
        graphCtx.beginPath();
        last.forEach((pt, i) => {
            const x = (i / (last.length-1)) * w;
            const y = h - (pt[s.key] / s.max) * h * 0.9 - h * 0.05;
            if (i === 0) graphCtx.moveTo(x, y);
            else graphCtx.lineTo(x, y);
        });
        graphCtx.stroke();
    });
}

// ─── Actions ───
async function doScan() {
    document.getElementById('scanOverlay').classList.add('active');
    document.getElementById('scanText').textContent = 'Scanning for DTCs...';
    const r = await api('scan');
    document.getElementById('scanOverlay').classList.remove('active');
    showPage('dtcs');
}

async function doClear() {
    if (!confirm('Clear all DTCs? This will reset the check engine light.')) return;
    document.getElementById('scanOverlay').classList.add('active');
    document.getElementById('scanText').textContent = 'Clearing DTCs...';
    await api('clear');
    document.getElementById('scanOverlay').classList.remove('active');
}

async function doConnect() {
    await api('connect');
}

async function doExport() {
    const r = await api('export');
    alert(r.message || 'Report saved');
}

async function sendRaw() {
    const hex = document.getElementById('rawCmd').value.trim();
    if (!hex) return;
    const r = await api('raw', {cmd: hex});
}

function quickCmd(hex) {
    document.getElementById('rawCmd').value = hex;
    sendRaw();
}

function clearLog() {
    document.getElementById('rawLog').innerHTML = '';
}

function escHtml(s) {
    return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

// Start polling
pollInterval = setInterval(poll, 500);
poll();
</script>
</body>
</html>"""

# ─── HTTP Handler ──────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress default logging

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(HTML_PAGE.encode())

    def do_POST(self):
        path = urlparse(self.path).path
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length)) if length > 0 else {}

        result = {}

        if path == "/api/state":
            # Build DTC info map
            dtc_info = {}
            for code in state["dtcs"]:
                if code in DTC_DB:
                    desc, fix, sev = DTC_DB[code]
                    dtc_info[code] = {"desc": desc, "fix": fix, "sev": sev}
                else:
                    dtc_info[code] = {"desc": "Unknown code - check service manual", "fix": "Look up this code for your specific vehicle", "sev": "Medium"}
            result = {**state, "dtc_info": dtc_info}

        elif path == "/api/scan":
            dtcs = cmd_scan_dtcs()
            result = {"dtcs": dtcs, "count": len(dtcs)}

        elif path == "/api/clear":
            cmd_clear_dtcs()
            result = {"ok": True}

        elif path == "/api/connect":
            close_serial()
            ok = open_serial()
            result = {"connected": ok}

        elif path == "/api/raw":
            frames = cmd_send_raw(body.get("cmd", ""))
            result = {"frames": frames}

        elif path == "/api/export":
            try:
                report = [
                    "MobileCLI OBD2 Pro Tuner Report",
                    f"Generated: {datetime.now()}",
                    "",
                    f"Head Unit: {state['head_unit']}",
                    f"Firmware: {state['firmware']}",
                    f"MCU ID: {state['mcu_id']}",
                    f"Port: {state['port']} @ {state['baud']}",
                    "",
                    f"MIL (Check Engine): {'ON' if state['mil'] else 'OFF'}",
                    f"DTC Count: {state['dtc_count']}",
                    "",
                ]
                if state["dtcs"]:
                    report.append("DIAGNOSTIC TROUBLE CODES:")
                    for i, code in enumerate(state["dtcs"]):
                        desc, fix, sev = DTC_DB.get(code, ("Unknown", "Consult manual", "?"))
                        report.append(f"  #{i+1} {code} [{sev}]")
                        report.append(f"      {desc}")
                        report.append(f"      FIX: {fix}")
                        report.append("")

                report.append("\nSCAN LOG:")
                report.extend(state["scan_log"][-50:])

                path = f"/sdcard/Download/OBD2_ProTuner_Report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
                with open(path, "w") as f:
                    f.write("\n".join(report))
                result = {"message": f"Saved to {path}"}
            except Exception as e:
                result = {"message": f"Error: {e}"}

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(json.dumps(result).encode())

# ─── Main ──────────────────────────────────────────────────

def main():
    global running

    print(f"""
\033[92m╔══════════════════════════════════════╗
║   OBD2 PRO TUNER — MobileCLI        ║
║   http://127.0.0.1:{PORT}             ║
╚══════════════════════════════════════╝\033[0m
""")

    # Open serial
    open_serial()

    # Start background reader
    reader = threading.Thread(target=background_reader, daemon=True)
    reader.start()

    # Start HTTP server
    server = HTTPServer(("0.0.0.0", PORT), Handler)
    log("Server started on port " + str(PORT))

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        running = False
        close_serial()
        server.shutdown()
        print("\nShutdown.")

if __name__ == "__main__":
    main()

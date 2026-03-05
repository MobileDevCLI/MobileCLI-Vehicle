# OBD2 Pro Tuner

**Real-time vehicle diagnostics for Android head units with hardwired OBD2 adapters.**

Built for the Szchoiceway GT6-CAR platform running MobileCLI. Communicates directly with the MCU via serial port — no Bluetooth adapter needed.

## Features

- **Web-based Pro Tuner UI** at `localhost:8099` — dark race-tuner theme
- **6-tab interface**: Dashboard, DTCs, Live Data, Raw Hex, Tools, Info
- **Real-time serial data streaming** from MCU at 5+ fps
- **DTC Scanner** with 100+ code database, severity levels, and fix instructions
- **Clear DTCs** — send Mode 04 to clear stored trouble codes
- **Raw hex command sender** with quick-command buttons for MCU protocol
- **Export diagnostic reports** to `/sdcard/Download/`
- **Android APK** (21KB) with native serial communication
- **CLI tools**: `obd2-scan` and `obd2-tuner`

## Tested Vehicle

- **2008 Toyota FJ Cruiser 4WD** (base model)
- CAN Profile: LuZheng / CARTYPE_TOYOTA_FJ_CRUISER

## Hardware

| Component | Details |
|-----------|---------|
| Head Unit | Szchoiceway GT6-CAR |
| Platform | Qualcomm SM6125, Android 13 |
| Firmware | GT6-EAU-T16.00.050 |
| MCU | PRLC0__GT6E_GB___S148 7808 |
| Serial | /dev/ttyHS1 @ 115200 baud (hardwired) |

## Quick Start

```bash
# Launch web UI
obd2-tuner

# CLI scanner
obd2-scan
obd2-scan --status    # Quick MIL/DTC check
obd2-scan --raw       # Raw serial dump

# Install APK
obd2-scan --app
```

## Web UI

The Pro Tuner web interface runs at `http://127.0.0.1:8099` and provides:

- **Dashboard** — MIL status, DTC count, connection state
- **DTCs** — Scan, view, clear diagnostic trouble codes
- **Live Data** — Real-time hex stream, frame statistics, heartbeat decode
- **Raw Hex** — Send arbitrary commands to MCU
- **Tools** — Scan vehicle, clear codes, export reports
- **Info** — Vehicle details, head unit info, CAN profile

## Protocol

Szchoiceway MCU communicates via CRLF-delimited binary frames:

| Frame Type | Bytes | Description |
|-----------|-------|-------------|
| 0x03 | 4 | Status |
| 0x04 | 5 | Heartbeat (MIL + DTC count) |
| 0x08 | 9 | Sensor/CAN relay data |
| 0x11 | 10+ | Extended response |
| 0x1D | 30+ | Identification |

Heartbeat `04 71 01 00 XX`: Bit 7 = MIL on/off, Bits 0-6 = DTC count.

## Build from Source

```bash
cd ~/obd2-scanner
bash build.sh
# Output: build/OBD2Scanner.apk
```

Requires: Java 17, aapt2, d8, apksigner, zipalign (all in MobileCLI/Termux).

## Files

| File | Description |
|------|-------------|
| `server.py` | Web UI server (full SPA embedded) |
| `build.sh` | APK build script |
| `app/src/main/java/.../OBD2Scanner.java` | Android activity |
| `app/src/main/AndroidManifest.xml` | APK manifest |

## Part of MobileCLI

This is the vehicle diagnostics module for [MobileCLI](https://github.com/MobileDevCLI/MobileCLI-v9.0.0) — the AI development environment for Android.

## License

MIT

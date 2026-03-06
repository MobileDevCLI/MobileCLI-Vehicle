package com.mobilecli.obd2;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import java.io.*;
import java.util.*;

/**
 * Bluetooth ELM327 OBD2 adapter connection and communication.
 * Handles scanning, pairing, connecting, and sending OBD2 PIDs.
 */
public class BluetoothOBD2 {

    private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    private BluetoothAdapter adapter;
    private BluetoothSocket socket;
    private InputStream in;
    private OutputStream out;
    private boolean connected = false;
    private StringBuilder logBuffer = new StringBuilder();

    public interface StatusCallback {
        void onStatus(String msg);
    }

    public BluetoothOBD2() {
        adapter = BluetoothAdapter.getDefaultAdapter();
    }

    public boolean isBluetoothAvailable() {
        return adapter != null && adapter.isEnabled();
    }

    public String getAdapterInfo() {
        if (adapter == null) return "No Bluetooth adapter";
        return "BT: " + adapter.getName() + " [" + adapter.getAddress() + "] " +
               (adapter.isEnabled() ? "ON" : "OFF");
    }

    /** Get paired devices, filtering for likely OBD2 adapters */
    public List<BluetoothDevice> getPairedDevices() {
        List<BluetoothDevice> result = new ArrayList<>();
        if (adapter == null) return result;
        Set<BluetoothDevice> bonded = adapter.getBondedDevices();
        if (bonded != null) {
            result.addAll(bonded);
        }
        return result;
    }

    /** Find the OBD2 adapter among paired devices */
    public BluetoothDevice findOBD2Device() {
        for (BluetoothDevice d : getPairedDevices()) {
            String name = d.getName();
            if (name != null) {
                String lower = name.toLowerCase();
                if (lower.contains("obd") || lower.contains("elm") || lower.contains("vlink") ||
                    lower.contains("scan") || lower.contains("v-link") || lower.contains("obdii") ||
                    lower.contains("torque") || lower.contains("car")) {
                    return d;
                }
            }
        }
        // If no named OBD device found, return first paired device (likely the OBD adapter)
        List<BluetoothDevice> paired = getPairedDevices();
        return paired.isEmpty() ? null : paired.get(0);
    }

    /** Start Bluetooth discovery for new devices */
    public boolean startDiscovery() {
        if (adapter == null) return false;
        adapter.cancelDiscovery();
        return adapter.startDiscovery();
    }

    public void cancelDiscovery() {
        if (adapter != null) adapter.cancelDiscovery();
    }

    /** Connect to a Bluetooth device via RFCOMM SPP */
    public boolean connect(BluetoothDevice device, StatusCallback cb) {
        try {
            if (adapter != null) adapter.cancelDiscovery();

            cb.onStatus("Connecting to " + device.getName() + " [" + device.getAddress() + "]...");

            // Try standard SPP UUID first
            socket = device.createRfcommSocketToServiceRecord(SPP_UUID);
            try {
                socket.connect();
            } catch (IOException e) {
                // Fallback: use reflection for channel 1
                cb.onStatus("SPP failed, trying fallback...");
                socket.close();
                socket = (BluetoothSocket) device.getClass()
                    .getMethod("createRfcommSocket", int.class)
                    .invoke(device, 1);
                socket.connect();
            }

            in = socket.getInputStream();
            out = socket.getOutputStream();
            connected = true;

            cb.onStatus("Connected! Initializing ELM327...");
            return true;
        } catch (Exception e) {
            cb.onStatus("Connect failed: " + e.getMessage());
            connected = false;
            return false;
        }
    }

    /** Initialize the ELM327 adapter */
    public String initELM327(StatusCallback cb) {
        StringBuilder info = new StringBuilder();
        try {
            // Reset
            String resp = sendCommand("ATZ", 2000);
            info.append("Reset: ").append(resp.trim()).append("\n");
            cb.onStatus("ELM327 reset OK");

            // Echo off
            sendCommand("ATE0", 500);

            // Get version
            resp = sendCommand("ATI", 1000);
            info.append("Version: ").append(resp.trim()).append("\n");

            // Linefeed off
            sendCommand("ATL0", 500);

            // Spaces off (for easier parsing)
            sendCommand("ATS0", 500);

            // Auto protocol detect
            resp = sendCommand("ATSP0", 1000);
            info.append("Protocol: Auto\n");

            // Set timeout
            sendCommand("ATST32", 500);

            // Headers off
            sendCommand("ATH0", 500);

            cb.onStatus("ELM327 initialized");
        } catch (Exception e) {
            info.append("Init error: ").append(e.getMessage()).append("\n");
        }
        return info.toString();
    }

    /** Send an OBD2 PID command and return the response */
    public String sendCommand(String cmd, int timeoutMs) throws IOException {
        if (!connected || out == null || in == null) {
            throw new IOException("Not connected");
        }

        // Flush input
        while (in.available() > 0) in.read();

        // Send command
        out.write((cmd + "\r").getBytes());
        out.flush();

        // Read response until we get the > prompt
        StringBuilder sb = new StringBuilder();
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < deadline) {
            if (in.available() > 0) {
                int b = in.read();
                if (b == '>') break;
                sb.append((char) b);
            } else {
                try { Thread.sleep(50); } catch (InterruptedException e) { break; }
            }
        }

        String response = sb.toString().trim();
        logBuffer.append(cmd).append(" => ").append(response).append("\n");
        return response;
    }

    /** Query a specific OBD2 PID and return raw hex response */
    public String queryPID(String pid) {
        try {
            String resp = sendCommand(pid, 3000);
            // Filter out "SEARCHING..." and "NO DATA" etc
            if (resp.contains("NO DATA") || resp.contains("UNABLE") || resp.contains("ERROR")) {
                return null;
            }
            return resp;
        } catch (Exception e) {
            return null;
        }
    }

    /** Get supported PIDs (Mode 01, PID 00) */
    public String getSupportedPIDs() {
        return queryPID("0100");
    }

    /** Get RPM */
    public int getRPM() {
        String resp = queryPID("010C");
        if (resp == null) return -1;
        try {
            String hex = cleanResponse(resp, "410C");
            if (hex.length() >= 4) {
                int a = Integer.parseInt(hex.substring(0, 2), 16);
                int b = Integer.parseInt(hex.substring(2, 4), 16);
                return (a * 256 + b) / 4;
            }
        } catch (Exception e) {}
        return -1;
    }

    /** Get Speed in km/h */
    public int getSpeed() {
        String resp = queryPID("010D");
        if (resp == null) return -1;
        try {
            String hex = cleanResponse(resp, "410D");
            if (hex.length() >= 2) {
                return Integer.parseInt(hex.substring(0, 2), 16);
            }
        } catch (Exception e) {}
        return -1;
    }

    /** Get coolant temp in C */
    public int getCoolantTemp() {
        String resp = queryPID("0105");
        if (resp == null) return -999;
        try {
            String hex = cleanResponse(resp, "4105");
            if (hex.length() >= 2) {
                return Integer.parseInt(hex.substring(0, 2), 16) - 40;
            }
        } catch (Exception e) {}
        return -999;
    }

    /** Get throttle position % */
    public int getThrottlePosition() {
        String resp = queryPID("0111");
        if (resp == null) return -1;
        try {
            String hex = cleanResponse(resp, "4111");
            if (hex.length() >= 2) {
                return Integer.parseInt(hex.substring(0, 2), 16) * 100 / 255;
            }
        } catch (Exception e) {}
        return -1;
    }

    /** Get engine load % */
    public int getEngineLoad() {
        String resp = queryPID("0104");
        if (resp == null) return -1;
        try {
            String hex = cleanResponse(resp, "4104");
            if (hex.length() >= 2) {
                return Integer.parseInt(hex.substring(0, 2), 16) * 100 / 255;
            }
        } catch (Exception e) {}
        return -1;
    }

    /** Get intake air temp in C */
    public int getIntakeAirTemp() {
        String resp = queryPID("010F");
        if (resp == null) return -999;
        try {
            String hex = cleanResponse(resp, "410F");
            if (hex.length() >= 2) {
                return Integer.parseInt(hex.substring(0, 2), 16) - 40;
            }
        } catch (Exception e) {}
        return -999;
    }

    /** Get battery voltage */
    public String getBatteryVoltage() {
        String resp = queryPID("ATRV");
        if (resp == null) return "N/A";
        return resp.replace("V", "").trim() + "V";
    }

    /** Get DTCs (Mode 03) */
    public List<String> getDTCs() {
        List<String> dtcs = new ArrayList<>();
        String resp = queryPID("03");
        if (resp == null) return dtcs;

        String hex = resp.replaceAll("[^0-9A-Fa-f]", "");
        // Response starts with 43, then pairs of bytes for each DTC
        int idx = hex.indexOf("43");
        if (idx < 0) return dtcs;
        hex = hex.substring(idx + 2);

        for (int i = 0; i + 3 < hex.length(); i += 4) {
            int b1 = Integer.parseInt(hex.substring(i, i + 2), 16);
            int b2 = Integer.parseInt(hex.substring(i + 2, i + 4), 16);
            if (b1 == 0 && b2 == 0) continue;

            String[] prefixes = {"P", "C", "B", "U"};
            String prefix = prefixes[(b1 >> 6) & 0x03];
            int code = ((b1 & 0x3F) << 8) | b2;
            dtcs.add(String.format("%s%04X", prefix, code));
        }
        return dtcs;
    }

    /** Clear DTCs (Mode 04) */
    public boolean clearDTCs() {
        try {
            String resp = sendCommand("04", 5000);
            return resp.contains("44") || !resp.contains("ERROR");
        } catch (Exception e) {
            return false;
        }
    }

    /** Get VIN */
    public String getVIN() {
        String resp = queryPID("0902");
        if (resp == null) return "N/A";
        // Parse multi-line VIN response
        String hex = resp.replaceAll("[^0-9A-Fa-f]", "");
        int idx = hex.indexOf("4902");
        if (idx < 0) return resp.trim();
        hex = hex.substring(idx);
        StringBuilder vin = new StringBuilder();
        // Skip header bytes and decode ASCII
        for (int i = 8; i + 1 < hex.length(); i += 2) {
            int ch = Integer.parseInt(hex.substring(i, i + 2), 16);
            if (ch >= 0x20 && ch <= 0x7E) vin.append((char) ch);
        }
        return vin.length() > 0 ? vin.toString() : resp.trim();
    }

    /** Get full vehicle scan data as formatted string */
    public String getFullScan(StatusCallback cb) {
        StringBuilder sb = new StringBuilder();
        sb.append("=== OBD2 Bluetooth Scan ===\n\n");

        cb.onStatus("Reading supported PIDs...");
        String pids = getSupportedPIDs();
        sb.append("Supported PIDs: ").append(pids != null ? pids : "N/A").append("\n\n");

        cb.onStatus("Reading RPM...");
        int rpm = getRPM();
        sb.append("RPM: ").append(rpm >= 0 ? rpm : "N/A").append("\n");

        cb.onStatus("Reading speed...");
        int speed = getSpeed();
        sb.append("Speed: ").append(speed >= 0 ? speed + " km/h" : "N/A").append("\n");

        cb.onStatus("Reading coolant temp...");
        int coolant = getCoolantTemp();
        sb.append("Coolant Temp: ").append(coolant > -999 ? coolant + " C" : "N/A").append("\n");

        cb.onStatus("Reading throttle...");
        int throttle = getThrottlePosition();
        sb.append("Throttle: ").append(throttle >= 0 ? throttle + "%" : "N/A").append("\n");

        cb.onStatus("Reading engine load...");
        int load = getEngineLoad();
        sb.append("Engine Load: ").append(load >= 0 ? load + "%" : "N/A").append("\n");

        cb.onStatus("Reading intake temp...");
        int intake = getIntakeAirTemp();
        sb.append("Intake Air Temp: ").append(intake > -999 ? intake + " C" : "N/A").append("\n");

        cb.onStatus("Reading battery voltage...");
        String volts = getBatteryVoltage();
        sb.append("Battery: ").append(volts).append("\n");

        cb.onStatus("Reading VIN...");
        String vin = getVIN();
        sb.append("VIN: ").append(vin).append("\n");

        cb.onStatus("Reading DTCs...");
        List<String> dtcs = getDTCs();
        sb.append("\nDTCs: ").append(dtcs.isEmpty() ? "None" : dtcs.size() + " found").append("\n");
        for (String dtc : dtcs) {
            sb.append("  ").append(dtc).append("\n");
        }

        sb.append("\n=== Scan Complete ===\n");
        return sb.toString();
    }

    /** Clean ELM327 response, extract data after expected header */
    private String cleanResponse(String resp, String header) {
        String clean = resp.replaceAll("[^0-9A-Fa-f]", "");
        int idx = clean.indexOf(header.toUpperCase());
        if (idx >= 0) {
            return clean.substring(idx + header.length());
        }
        idx = clean.indexOf(header.toLowerCase());
        if (idx >= 0) {
            return clean.substring(idx + header.length());
        }
        return clean;
    }

    public boolean isConnected() {
        return connected;
    }

    public String getLog() {
        return logBuffer.toString();
    }

    public void disconnect() {
        connected = false;
        try {
            if (in != null) in.close();
            if (out != null) out.close();
            if (socket != null) socket.close();
        } catch (Exception e) {}
    }
}

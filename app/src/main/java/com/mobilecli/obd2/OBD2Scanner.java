package com.mobilecli.obd2;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.widget.TextView;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.view.Gravity;
import android.view.View;
import android.graphics.Color;
import android.graphics.Typeface;
import android.util.TypedValue;
import java.io.*;
import java.util.*;

/**
 * MobileCLI OBD2 Scanner
 * Reads DTCs and vehicle data via Szchoiceway MCU serial protocol.
 * Communicates directly with /dev/ttyHS1 at 115200 baud.
 */
public class OBD2Scanner extends Activity {

    private TextView statusText;
    private TextView dtcText;
    private TextView vehicleInfoText;
    private ScrollView scrollView;
    private ProgressBar progressBar;
    private Handler handler;
    private FileInputStream serialIn;
    private FileOutputStream serialOut;
    private volatile boolean scanning = false;

    // MCU protocol constants
    private static final String SERIAL_PORT = "/dev/ttyHS1";
    private static final int BAUD_RATE = 115200;
    private static final byte[] HEADER_55 = {0x55};
    private static final byte[] HEARTBEAT = {0x04, 0x71, 0x01, 0x00, (byte)0x89};

    // DTC type prefixes
    private static final String[] DTC_PREFIX = {"P", "C", "B", "U"};

    // Common DTC descriptions database
    private static final Map<String, String> DTC_DB = new HashMap<>();
    static {
        // Powertrain - Generic
        DTC_DB.put("P0010", "Intake Camshaft Position Actuator Circuit (Bank 1)");
        DTC_DB.put("P0011", "Intake Camshaft Position Timing Over-Advanced (Bank 1)");
        DTC_DB.put("P0012", "Intake Camshaft Position Timing Over-Retarded (Bank 1)");
        DTC_DB.put("P0013", "Exhaust Camshaft Position Actuator Circuit (Bank 1)");
        DTC_DB.put("P0014", "Exhaust Camshaft Position Timing Over-Advanced (Bank 1)");
        DTC_DB.put("P0015", "Exhaust Camshaft Position Timing Over-Retarded (Bank 1)");
        DTC_DB.put("P0016", "Crankshaft/Camshaft Position Correlation (Bank 1 Sensor A)");
        DTC_DB.put("P0017", "Crankshaft/Camshaft Position Correlation (Bank 1 Sensor B)");
        DTC_DB.put("P0018", "Crankshaft/Camshaft Position Correlation (Bank 2 Sensor A)");
        DTC_DB.put("P0019", "Crankshaft/Camshaft Position Correlation (Bank 2 Sensor B)");
        DTC_DB.put("P0020", "Intake Camshaft Position Actuator Circuit (Bank 2)");
        DTC_DB.put("P0021", "Intake Camshaft Position Timing Over-Advanced (Bank 2)");
        DTC_DB.put("P0022", "Intake Camshaft Position Timing Over-Retarded (Bank 2)");
        DTC_DB.put("P0030", "HO2S Heater Control Circuit (Bank 1 Sensor 1)");
        DTC_DB.put("P0100", "Mass or Volume Air Flow Circuit Malfunction");
        DTC_DB.put("P0101", "Mass or Volume Air Flow Circuit Range/Performance");
        DTC_DB.put("P0102", "Mass or Volume Air Flow Circuit Low Input");
        DTC_DB.put("P0103", "Mass or Volume Air Flow Circuit High Input");
        DTC_DB.put("P0106", "MAP/Barometric Pressure Circuit Range/Performance");
        DTC_DB.put("P0107", "MAP/Barometric Pressure Circuit Low Input");
        DTC_DB.put("P0108", "MAP/Barometric Pressure Circuit High Input");
        DTC_DB.put("P0110", "Intake Air Temperature Circuit Malfunction");
        DTC_DB.put("P0111", "Intake Air Temperature Circuit Range/Performance");
        DTC_DB.put("P0112", "Intake Air Temperature Circuit Low Input");
        DTC_DB.put("P0113", "Intake Air Temperature Circuit High Input");
        DTC_DB.put("P0115", "Engine Coolant Temperature Circuit Malfunction");
        DTC_DB.put("P0116", "Engine Coolant Temperature Circuit Range/Performance");
        DTC_DB.put("P0117", "Engine Coolant Temperature Circuit Low Input");
        DTC_DB.put("P0118", "Engine Coolant Temperature Circuit High Input");
        DTC_DB.put("P0120", "Throttle Position Sensor Circuit Malfunction");
        DTC_DB.put("P0121", "Throttle Position Sensor Circuit Range/Performance");
        DTC_DB.put("P0122", "Throttle Position Sensor Circuit Low Input");
        DTC_DB.put("P0123", "Throttle Position Sensor Circuit High Input");
        DTC_DB.put("P0125", "Insufficient Coolant Temperature for Closed Loop Fuel Control");
        DTC_DB.put("P0128", "Coolant Thermostat Below Thermostat Regulating Temperature");
        DTC_DB.put("P0130", "O2 Sensor Circuit Malfunction (Bank 1 Sensor 1)");
        DTC_DB.put("P0131", "O2 Sensor Circuit Low Voltage (Bank 1 Sensor 1)");
        DTC_DB.put("P0132", "O2 Sensor Circuit High Voltage (Bank 1 Sensor 1)");
        DTC_DB.put("P0133", "O2 Sensor Circuit Slow Response (Bank 1 Sensor 1)");
        DTC_DB.put("P0134", "O2 Sensor Circuit No Activity (Bank 1 Sensor 1)");
        DTC_DB.put("P0135", "O2 Sensor Heater Circuit Malfunction (Bank 1 Sensor 1)");
        DTC_DB.put("P0136", "O2 Sensor Circuit Malfunction (Bank 1 Sensor 2)");
        DTC_DB.put("P0137", "O2 Sensor Circuit Low Voltage (Bank 1 Sensor 2)");
        DTC_DB.put("P0138", "O2 Sensor Circuit High Voltage (Bank 1 Sensor 2)");
        DTC_DB.put("P0139", "O2 Sensor Circuit Slow Response (Bank 1 Sensor 2)");
        DTC_DB.put("P0140", "O2 Sensor Circuit No Activity (Bank 1 Sensor 2)");
        DTC_DB.put("P0141", "O2 Sensor Heater Circuit Malfunction (Bank 1 Sensor 2)");
        DTC_DB.put("P0171", "System Too Lean (Bank 1)");
        DTC_DB.put("P0172", "System Too Rich (Bank 1)");
        DTC_DB.put("P0174", "System Too Lean (Bank 2)");
        DTC_DB.put("P0175", "System Too Rich (Bank 2)");
        DTC_DB.put("P0200", "Injector Circuit Malfunction");
        DTC_DB.put("P0217", "Engine Overtemp Condition");
        DTC_DB.put("P0218", "Transmission Over Temperature Condition");
        DTC_DB.put("P0219", "Engine Overspeed Condition");
        DTC_DB.put("P0300", "Random/Multiple Cylinder Misfire Detected");
        DTC_DB.put("P0301", "Cylinder 1 Misfire Detected");
        DTC_DB.put("P0302", "Cylinder 2 Misfire Detected");
        DTC_DB.put("P0303", "Cylinder 3 Misfire Detected");
        DTC_DB.put("P0304", "Cylinder 4 Misfire Detected");
        DTC_DB.put("P0305", "Cylinder 5 Misfire Detected");
        DTC_DB.put("P0306", "Cylinder 6 Misfire Detected");
        DTC_DB.put("P0325", "Knock Sensor 1 Circuit Malfunction (Bank 1)");
        DTC_DB.put("P0335", "Crankshaft Position Sensor A Circuit Malfunction");
        DTC_DB.put("P0336", "Crankshaft Position Sensor A Circuit Range/Performance");
        DTC_DB.put("P0340", "Camshaft Position Sensor Circuit Malfunction (Bank 1)");
        DTC_DB.put("P0341", "Camshaft Position Sensor Circuit Range/Performance (Bank 1)");
        DTC_DB.put("P0400", "Exhaust Gas Recirculation Flow Malfunction");
        DTC_DB.put("P0401", "EGR Flow Insufficient Detected");
        DTC_DB.put("P0402", "EGR Flow Excessive Detected");
        DTC_DB.put("P0410", "Secondary Air Injection System Malfunction");
        DTC_DB.put("P0411", "Secondary Air Injection System Incorrect Flow Detected");
        DTC_DB.put("P0420", "Catalyst System Efficiency Below Threshold (Bank 1)");
        DTC_DB.put("P0421", "Warm Up Catalyst Efficiency Below Threshold (Bank 1)");
        DTC_DB.put("P0430", "Catalyst System Efficiency Below Threshold (Bank 2)");
        DTC_DB.put("P0440", "Evaporative Emission Control System Malfunction");
        DTC_DB.put("P0441", "EVAP System Incorrect Purge Flow");
        DTC_DB.put("P0442", "EVAP System Leak Detected (small leak)");
        DTC_DB.put("P0443", "EVAP System Purge Control Valve Circuit Malfunction");
        DTC_DB.put("P0446", "EVAP Vent Control Circuit Malfunction");
        DTC_DB.put("P0449", "EVAP Vent Valve/Solenoid Circuit Malfunction");
        DTC_DB.put("P0455", "EVAP System Leak Detected (large leak)");
        DTC_DB.put("P0456", "EVAP System Leak Detected (very small leak)");
        DTC_DB.put("P0461", "Fuel Level Sensor Circuit Range/Performance");
        DTC_DB.put("P0500", "Vehicle Speed Sensor Malfunction");
        DTC_DB.put("P0505", "Idle Control System Malfunction");
        DTC_DB.put("P0506", "Idle Control System RPM Lower Than Expected");
        DTC_DB.put("P0507", "Idle Control System RPM Higher Than Expected");
        DTC_DB.put("P0562", "System Voltage Low");
        DTC_DB.put("P0563", "System Voltage High");
        DTC_DB.put("P0600", "Serial Communication Link Malfunction");
        DTC_DB.put("P0601", "Internal Control Module Memory Check Sum Error");
        DTC_DB.put("P0602", "Control Module Programming Error");
        DTC_DB.put("P0606", "PCM Processor Fault");
        DTC_DB.put("P0700", "Transmission Control System Malfunction");
        DTC_DB.put("P0706", "Transmission Range Sensor Circuit Range/Performance");
        DTC_DB.put("P0715", "Input/Turbine Speed Sensor Circuit Malfunction");
        DTC_DB.put("P0720", "Output Speed Sensor Circuit Malfunction");
        DTC_DB.put("P0725", "Engine Speed Input Circuit Malfunction");
        DTC_DB.put("P0740", "Torque Converter Clutch Circuit Malfunction");
        DTC_DB.put("P0741", "Torque Converter Clutch Circuit Stuck Off");
        DTC_DB.put("P0750", "Shift Solenoid A Malfunction");
        DTC_DB.put("P0755", "Shift Solenoid B Malfunction");
        // Body
        DTC_DB.put("B0001", "Driver Frontal Stage 1 Deployment Control");
        DTC_DB.put("B0100", "Electronic Frontal Sensor 1 Malfunction");
        // Chassis
        DTC_DB.put("C0035", "Left Front Wheel Speed Circuit Malfunction");
        DTC_DB.put("C0040", "Right Front Wheel Speed Circuit Malfunction");
        DTC_DB.put("C0045", "Left Rear Wheel Speed Circuit Malfunction");
        DTC_DB.put("C0050", "Right Rear Wheel Speed Circuit Malfunction");
        DTC_DB.put("C0060", "Left Front ABS Solenoid 1 Circuit Malfunction");
        // Network
        DTC_DB.put("U0001", "High Speed CAN Communication Bus");
        DTC_DB.put("U0073", "Control Module Communication Bus Off");
        DTC_DB.put("U0100", "Lost Communication with ECM/PCM");
        DTC_DB.put("U0101", "Lost Communication with TCM");
        DTC_DB.put("U0121", "Lost Communication with ABS");
        DTC_DB.put("U0140", "Lost Communication with BCM");
        DTC_DB.put("U0155", "Lost Communication with Instrument Cluster");
        DTC_DB.put("U0164", "Lost Communication with HVAC");
        DTC_DB.put("U0401", "Invalid Data Received from ECM/PCM");
        DTC_DB.put("U0402", "Invalid Data Received from TCM");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        handler = new Handler(Looper.getMainLooper());

        // Build UI programmatically (no XML dependency)
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(Color.parseColor("#1a1a2e"));
        root.setPadding(24, 48, 24, 24);

        // Title
        TextView title = new TextView(this);
        title.setText("MobileCLI OBD2 Scanner");
        title.setTextColor(Color.parseColor("#00ff88"));
        title.setTextSize(TypedValue.COMPLEX_UNIT_SP, 24);
        title.setTypeface(Typeface.MONOSPACE, Typeface.BOLD);
        title.setGravity(Gravity.CENTER);
        title.setPadding(0, 0, 0, 16);
        root.addView(title);

        // Status
        statusText = new TextView(this);
        statusText.setText("Ready to scan...");
        statusText.setTextColor(Color.parseColor("#e0e0e0"));
        statusText.setTextSize(TypedValue.COMPLEX_UNIT_SP, 14);
        statusText.setTypeface(Typeface.MONOSPACE);
        statusText.setPadding(0, 0, 0, 8);
        root.addView(statusText);

        // Progress
        progressBar = new ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal);
        progressBar.setMax(100);
        progressBar.setProgress(0);
        progressBar.setVisibility(View.GONE);
        root.addView(progressBar);

        // Scan button
        Button scanBtn = new Button(this);
        scanBtn.setText("SCAN VEHICLE");
        scanBtn.setBackgroundColor(Color.parseColor("#00ff88"));
        scanBtn.setTextColor(Color.BLACK);
        scanBtn.setTypeface(Typeface.MONOSPACE, Typeface.BOLD);
        scanBtn.setPadding(16, 16, 16, 16);
        scanBtn.setOnClickListener(v -> startScan());
        root.addView(scanBtn);

        // Clear DTC button
        Button clearBtn = new Button(this);
        clearBtn.setText("CLEAR CODES");
        clearBtn.setBackgroundColor(Color.parseColor("#ff4444"));
        clearBtn.setTextColor(Color.WHITE);
        clearBtn.setTypeface(Typeface.MONOSPACE, Typeface.BOLD);
        clearBtn.setPadding(16, 8, 16, 8);
        LinearLayout.LayoutParams clearParams = new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        clearParams.topMargin = 8;
        clearBtn.setLayoutParams(clearParams);
        clearBtn.setOnClickListener(v -> clearDTCs());
        root.addView(clearBtn);

        // Vehicle info section
        vehicleInfoText = new TextView(this);
        vehicleInfoText.setText("");
        vehicleInfoText.setTextColor(Color.parseColor("#88ccff"));
        vehicleInfoText.setTextSize(TypedValue.COMPLEX_UNIT_SP, 13);
        vehicleInfoText.setTypeface(Typeface.MONOSPACE);
        vehicleInfoText.setPadding(0, 16, 0, 8);
        root.addView(vehicleInfoText);

        // DTC results in scrollview
        scrollView = new ScrollView(this);
        scrollView.setLayoutParams(new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, 0, 1.0f));

        dtcText = new TextView(this);
        dtcText.setText("Press SCAN VEHICLE to begin diagnostics.");
        dtcText.setTextColor(Color.parseColor("#ffcc00"));
        dtcText.setTextSize(TypedValue.COMPLEX_UNIT_SP, 13);
        dtcText.setTypeface(Typeface.MONOSPACE);
        dtcText.setPadding(0, 8, 0, 8);
        scrollView.addView(dtcText);
        root.addView(scrollView);

        // Save button
        Button saveBtn = new Button(this);
        saveBtn.setText("SAVE REPORT");
        saveBtn.setBackgroundColor(Color.parseColor("#4488ff"));
        saveBtn.setTextColor(Color.WHITE);
        saveBtn.setTypeface(Typeface.MONOSPACE, Typeface.BOLD);
        LinearLayout.LayoutParams saveParams = new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        saveParams.topMargin = 8;
        saveBtn.setLayoutParams(saveParams);
        saveBtn.setOnClickListener(v -> saveReport());
        root.addView(saveBtn);

        setContentView(root);

        // Auto-scan on launch if started with intent
        if (getIntent() != null && getIntent().getBooleanExtra("auto_scan", false)) {
            startScan();
        }
    }

    private void startScan() {
        if (scanning) return;
        scanning = true;
        progressBar.setVisibility(View.VISIBLE);
        progressBar.setProgress(0);
        statusText.setText("Connecting to OBD2...");
        dtcText.setText("");

        new Thread(() -> {
            try {
                performScan();
            } catch (Exception e) {
                updateUI("Error: " + e.getMessage());
            } finally {
                scanning = false;
                handler.post(() -> progressBar.setVisibility(View.GONE));
            }
        }).start();
    }

    private void performScan() {
        updateStatus("Opening serial port...");
        updateProgress(5);

        try {
            // Configure serial port via stty
            Runtime.getRuntime().exec(new String[]{
                "/system/bin/sh", "-c",
                "stty -F " + SERIAL_PORT + " 115200 raw -echo"
            }).waitFor();

            File portFile = new File(SERIAL_PORT);
            serialIn = new FileInputStream(portFile);
            serialOut = new FileOutputStream(portFile);

            updateProgress(10);
            updateStatus("Serial port open. Reading MCU status...");

            // Step 1: Read heartbeat to confirm connection
            byte[] heartbeat = readMCUHeartbeat();
            if (heartbeat != null && heartbeat.length >= 5) {
                int milOn = (heartbeat[4] >> 7) & 1;
                int dtcCount = heartbeat[4] & 0x7F;

                String milStatus = milOn == 1 ? "ON" : "OFF";
                updateVehicleInfo(
                    "Head Unit: Szchoiceway GT6-CAR\n" +
                    "MCU Port: " + SERIAL_PORT + " @ 115200\n" +
                    "MIL (Check Engine): " + milStatus + "\n" +
                    "Stored DTCs: " + dtcCount
                );
                updateProgress(25);

                if (dtcCount > 0) {
                    updateStatus("Found " + dtcCount + " DTCs. Reading codes...");
                    // Step 2: Try to read DTCs via MCU commands
                    List<String> dtcs = readDTCsFromMCU(dtcCount);
                    updateProgress(60);

                    // Step 3: If MCU didn't give full codes, try CAN bus service
                    if (dtcs.size() < dtcCount) {
                        updateStatus("Trying CAN bus service for remaining codes...");
                        List<String> canDtcs = readDTCsViaCanbusService();
                        for (String d : canDtcs) {
                            if (!dtcs.contains(d)) dtcs.add(d);
                        }
                    }
                    updateProgress(80);

                    // Step 4: Display results
                    displayDTCs(dtcs, dtcCount);
                    updateProgress(100);
                    updateStatus("Scan complete. " + dtcs.size() + " codes read.");
                } else {
                    updateStatus("No DTCs found. Vehicle is healthy!");
                    appendDTC("No diagnostic trouble codes stored.\nCheck engine light is OFF.");
                    updateProgress(100);
                }
            } else {
                updateStatus("Could not read MCU heartbeat. Trying direct CAN...");
                // Fallback: try direct OBD2 protocol
                readDTCsDirect();
                updateProgress(100);
            }

            serialIn.close();
            serialOut.close();

        } catch (Exception e) {
            updateStatus("Error: " + e.getMessage());
            appendDTC("\nFailed to communicate with OBD2 port.\n" +
                      "Error: " + e.getMessage() + "\n\n" +
                      "Troubleshooting:\n" +
                      "1. Check OBD2 adapter connection\n" +
                      "2. Ensure ignition is ON\n" +
                      "3. Try: chmod 666 /dev/ttyHS1");
        }
    }

    private byte[] readMCUHeartbeat() throws Exception {
        // Read data for 5 seconds looking for 0x04 heartbeat frame
        long start = System.currentTimeMillis();
        ByteArrayOutputStream buffer = new ByteArrayOutputStream();

        while (System.currentTimeMillis() - start < 5000) {
            if (serialIn.available() > 0) {
                byte[] chunk = new byte[serialIn.available()];
                serialIn.read(chunk);
                buffer.write(chunk);
            }
            Thread.sleep(100);
        }

        byte[] data = buffer.toByteArray();
        // Find 0x04 0x71 frames
        for (int i = 0; i < data.length - 4; i++) {
            if (data[i] == 0x0D && data[i+1] == 0x0A && data[i+2] == 0x04 && data[i+3] == 0x71) {
                byte[] frame = new byte[5];
                System.arraycopy(data, i+2, frame, 0, 5);
                return frame;
            }
        }
        return null;
    }

    private List<String> readDTCsFromMCU(int expectedCount) {
        List<String> dtcs = new ArrayList<>();
        try {
            // Send MCU commands to get DTC data
            // Command 0x60 (Warning Info) returns DTC-related data
            byte[][] commands = {
                {0x55, 0x04, 0x60, 0x00, 0x00, 0x01, 0x61},
                {0x55, 0x04, 0x60, 0x01, 0x00, 0x01, 0x60},
                {0x55, 0x04, 0x60, 0x02, 0x00, 0x01, 0x63},
                {0x55, 0x04, 0x70, 0x00, 0x00, 0x01, 0x71},
                {0x55, 0x04, 0x70, 0x01, 0x00, 0x01, 0x70},
                {0x55, 0x04, 0x70, 0x02, 0x00, 0x01, 0x73},
                {0x55, 0x04, 0x50, 0x00, 0x00, 0x01, 0x51},
            };

            for (byte[] cmd : commands) {
                serialOut.write(cmd);
                serialOut.flush();
                Thread.sleep(1500);

                if (serialIn.available() > 0) {
                    byte[] resp = new byte[serialIn.available()];
                    serialIn.read(resp);

                    // Parse response for extended frames (type 0x11)
                    List<byte[]> frames = parseFrames(resp);
                    for (byte[] frame : frames) {
                        if (frame[0] == 0x11 && frame.length >= 10) {
                            // Extract DTC data from extended frame
                            for (int i = 7; i < frame.length - 3; i += 2) {
                                int b1 = frame[i] & 0xFF;
                                int b2 = frame[i+1] & 0xFF;
                                if (b1 != 0 || b2 != 0) {
                                    String dtcType = DTC_PREFIX[(b1 >> 6) & 0x03];
                                    int dtcNum = ((b1 & 0x3F) << 8) | b2;
                                    String code = String.format("%s%04d", dtcType, dtcNum);
                                    if (!dtcs.contains(code)) {
                                        dtcs.add(code);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } catch (Exception e) {
            appendDTC("MCU read error: " + e.getMessage());
        }
        return dtcs;
    }

    private List<String> readDTCsViaCanbusService() {
        List<String> dtcs = new ArrayList<>();
        try {
            // Try to start canbus service and read DTCs via intent
            Runtime.getRuntime().exec(new String[]{
                "am", "startservice", "-n",
                "com.szchoiceway.zxwsyscanbus/.zxwCanbusService"
            });
            Thread.sleep(2000);

            // Also try to read from canbus2 content provider
            Process p = Runtime.getRuntime().exec(new String[]{
                "content", "query", "--uri",
                "content://com.szchoiceway.canbus2.provider/dtc"
            });
            BufferedReader reader = new BufferedReader(new InputStreamReader(p.getInputStream()));
            String line;
            while ((line = reader.readLine()) != null) {
                if (line.contains("dtc_code=")) {
                    String code = line.split("dtc_code=")[1].split("[,}]")[0].trim();
                    if (!dtcs.contains(code)) dtcs.add(code);
                }
            }
        } catch (Exception e) {
            // Silent - fallback will handle
        }
        return dtcs;
    }

    private void readDTCsDirect() {
        updateStatus("Direct OBD2 scan via serial...");
        try {
            // Send standard OBD2 Mode 03 (Read DTCs) wrapped in 0x55 protocol
            byte[] mode03 = {0x55, 0x02, 0x03, 0x00, 0x03};
            serialOut.write(mode03);
            serialOut.flush();
            Thread.sleep(3000);

            if (serialIn.available() > 0) {
                byte[] resp = new byte[serialIn.available()];
                serialIn.read(resp);
                appendDTC("Direct scan response: " + bytesToHex(resp));
            }
        } catch (Exception e) {
            appendDTC("Direct scan failed: " + e.getMessage());
        }
    }

    private void clearDTCs() {
        if (scanning) return;
        scanning = true;
        statusText.setText("Clearing DTCs...");

        new Thread(() -> {
            try {
                Runtime.getRuntime().exec(new String[]{
                    "/system/bin/sh", "-c",
                    "stty -F " + SERIAL_PORT + " 115200 raw -echo"
                }).waitFor();

                File portFile = new File(SERIAL_PORT);
                FileOutputStream out = new FileOutputStream(portFile);

                // Send OBD2 Mode 04 (Clear DTCs) via MCU
                byte[] clearCmd = {0x55, 0x02, 0x04, 0x00, 0x04};
                out.write(clearCmd);
                out.flush();
                Thread.sleep(2000);

                // Also try direct Mode 04
                byte[] mode04 = {0x55, 0x01, 0x04, 0x04};
                out.write(mode04);
                out.flush();
                out.close();

                handler.post(() -> {
                    statusText.setText("Clear command sent. Re-scan to verify.");
                    dtcText.setText("DTCs cleared. Please re-scan the vehicle.\n" +
                                   "Note: If codes return, the underlying issue still exists.");
                });
            } catch (Exception e) {
                handler.post(() -> statusText.setText("Clear failed: " + e.getMessage()));
            } finally {
                scanning = false;
            }
        }).start();
    }

    private void displayDTCs(List<String> dtcs, int expectedCount) {
        StringBuilder sb = new StringBuilder();
        sb.append("========== DTC REPORT ==========\n\n");

        if (dtcs.isEmpty()) {
            sb.append("MCU reports ").append(expectedCount).append(" DTCs stored\n");
            sb.append("but individual codes could not be extracted\n");
            sb.append("via the current serial protocol.\n\n");
            sb.append("The CAN bus services need to be running\n");
            sb.append("for full code enumeration.\n\n");
            sb.append("RECOMMENDED: Use a Bluetooth ELM327\n");
            sb.append("adapter with Torque or Car Scanner app\n");
            sb.append("to read the specific codes.\n");
        } else {
            for (int i = 0; i < dtcs.size(); i++) {
                String code = dtcs.get(i);
                String desc = DTC_DB.getOrDefault(code, "Unknown - Check vehicle service manual");
                sb.append(String.format("#%d  %s\n", i + 1, code));
                sb.append(String.format("    %s\n\n", desc));
            }

            if (dtcs.size() < expectedCount) {
                sb.append(String.format("\n[%d of %d codes read]\n", dtcs.size(), expectedCount));
                sb.append("Some codes may require CAN bus service.\n");
            }
        }

        sb.append("\n================================\n");
        sb.append("MIL (Check Engine Light): ON\n");
        sb.append("Total Stored Codes: ").append(expectedCount).append("\n");
        sb.append("Scan Time: ").append(new Date()).append("\n");

        final String result = sb.toString();
        handler.post(() -> dtcText.setText(result));
    }

    private void saveReport() {
        try {
            String report = "MobileCLI OBD2 Diagnostic Report\n" +
                "Generated: " + new Date() + "\n\n" +
                vehicleInfoText.getText() + "\n\n" +
                dtcText.getText();

            File file = new File("/sdcard/Download/OBD2_Diagnostic_Report.txt");
            FileWriter writer = new FileWriter(file);
            writer.write(report);
            writer.close();

            statusText.setText("Report saved to /sdcard/Download/OBD2_Diagnostic_Report.txt");
        } catch (Exception e) {
            statusText.setText("Save failed: " + e.getMessage());
        }
    }

    private List<byte[]> parseFrames(byte[] data) {
        List<byte[]> frames = new ArrayList<>();
        int i = 0;
        while (i < data.length - 1) {
            if (data[i] == 0x0D && data[i+1] == 0x0A && i + 2 < data.length) {
                int start = i + 2;
                // Find next \r\n
                int end = data.length;
                for (int j = start; j < data.length - 1; j++) {
                    if (data[j] == 0x0D && data[j+1] == 0x0A) {
                        end = j;
                        break;
                    }
                }
                if (end > start) {
                    byte[] frame = new byte[end - start];
                    System.arraycopy(data, start, frame, 0, end - start);
                    frames.add(frame);
                }
                i = end;
            } else {
                i++;
            }
        }
        return frames;
    }

    private void updateStatus(final String msg) {
        handler.post(() -> statusText.setText(msg));
    }

    private void updateVehicleInfo(final String info) {
        handler.post(() -> vehicleInfoText.setText(info));
    }

    private void appendDTC(final String text) {
        handler.post(() -> dtcText.append(text + "\n"));
    }

    private void updateUI(final String text) {
        handler.post(() -> dtcText.setText(text));
    }

    private void updateProgress(final int value) {
        handler.post(() -> progressBar.setProgress(value));
    }

    private static String bytesToHex(byte[] bytes) {
        StringBuilder sb = new StringBuilder();
        for (byte b : bytes) sb.append(String.format("%02X ", b & 0xFF));
        return sb.toString().trim();
    }
}

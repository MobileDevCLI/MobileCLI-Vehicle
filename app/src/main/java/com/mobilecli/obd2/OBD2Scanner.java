package com.mobilecli.obd2;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
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

public class OBD2Scanner extends Activity {

    private TextView statusText;
    private TextView dtcText;
    private TextView vehicleInfoText;
    private ScrollView scrollView;
    private ProgressBar progressBar;
    private Handler handler;
    private volatile boolean scanning = false;

    private BluetoothOBD2 btOBD2;
    private List<BluetoothDevice> discoveredDevices = new ArrayList<>();
    private LinearLayout deviceListLayout;

    // DTC descriptions
    private static final Map<String, String> DTC_DB = new HashMap<>();
    static {
        DTC_DB.put("P0010", "Intake Camshaft Position Actuator Circuit (Bank 1)");
        DTC_DB.put("P0011", "Intake Camshaft Position Timing Over-Advanced (Bank 1)");
        DTC_DB.put("P0100", "Mass or Volume Air Flow Circuit Malfunction");
        DTC_DB.put("P0101", "Mass or Volume Air Flow Circuit Range/Performance");
        DTC_DB.put("P0102", "Mass or Volume Air Flow Circuit Low Input");
        DTC_DB.put("P0103", "Mass or Volume Air Flow Circuit High Input");
        DTC_DB.put("P0110", "Intake Air Temperature Circuit Malfunction");
        DTC_DB.put("P0115", "Engine Coolant Temperature Circuit Malfunction");
        DTC_DB.put("P0120", "Throttle Position Sensor Circuit Malfunction");
        DTC_DB.put("P0125", "Insufficient Coolant Temp for Closed Loop");
        DTC_DB.put("P0128", "Coolant Thermostat Below Regulating Temp");
        DTC_DB.put("P0130", "O2 Sensor Circuit (Bank 1 Sensor 1)");
        DTC_DB.put("P0135", "O2 Sensor Heater Circuit (Bank 1 Sensor 1)");
        DTC_DB.put("P0171", "System Too Lean (Bank 1)");
        DTC_DB.put("P0172", "System Too Rich (Bank 1)");
        DTC_DB.put("P0174", "System Too Lean (Bank 2)");
        DTC_DB.put("P0175", "System Too Rich (Bank 2)");
        DTC_DB.put("P0300", "Random/Multiple Cylinder Misfire");
        DTC_DB.put("P0301", "Cylinder 1 Misfire");
        DTC_DB.put("P0302", "Cylinder 2 Misfire");
        DTC_DB.put("P0303", "Cylinder 3 Misfire");
        DTC_DB.put("P0304", "Cylinder 4 Misfire");
        DTC_DB.put("P0305", "Cylinder 5 Misfire");
        DTC_DB.put("P0306", "Cylinder 6 Misfire");
        DTC_DB.put("P0325", "Knock Sensor 1 Circuit (Bank 1)");
        DTC_DB.put("P0335", "Crankshaft Position Sensor A Circuit");
        DTC_DB.put("P0340", "Camshaft Position Sensor Circuit (Bank 1)");
        DTC_DB.put("P0400", "EGR Flow Malfunction");
        DTC_DB.put("P0420", "Catalyst Efficiency Below Threshold (Bank 1)");
        DTC_DB.put("P0430", "Catalyst Efficiency Below Threshold (Bank 2)");
        DTC_DB.put("P0440", "EVAP System Malfunction");
        DTC_DB.put("P0442", "EVAP System Leak (small)");
        DTC_DB.put("P0446", "EVAP Vent Control Circuit");
        DTC_DB.put("P0455", "EVAP System Leak (large)");
        DTC_DB.put("P0500", "Vehicle Speed Sensor");
        DTC_DB.put("P0505", "Idle Control System");
        DTC_DB.put("P0562", "System Voltage Low");
        DTC_DB.put("P0700", "Transmission Control System");
        DTC_DB.put("P0715", "Input/Turbine Speed Sensor Circuit");
        DTC_DB.put("P0720", "Output Speed Sensor Circuit");
        DTC_DB.put("P0740", "Torque Converter Clutch Circuit");
        DTC_DB.put("P0741", "Torque Converter Clutch Stuck Off");
        DTC_DB.put("U0001", "High Speed CAN Communication Bus");
        DTC_DB.put("U0100", "Lost Communication with ECM/PCM");
        DTC_DB.put("U0101", "Lost Communication with TCM");
        DTC_DB.put("U0121", "Lost Communication with ABS");
    }

    private BroadcastReceiver btReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (device != null && !discoveredDevices.contains(device)) {
                    discoveredDevices.add(device);
                    handler.post(() -> addDeviceButton(device));
                }
            } else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action)) {
                handler.post(() -> statusText.setText("Discovery complete. " + discoveredDevices.size() + " devices found."));
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        handler = new Handler(Looper.getMainLooper());
        btOBD2 = new BluetoothOBD2();

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(Color.parseColor("#1a1a2e"));
        root.setPadding(24, 48, 24, 24);

        // Title
        TextView title = new TextView(this);
        title.setText("OBD2 Pro Tuner - Bluetooth");
        title.setTextColor(Color.parseColor("#00ff88"));
        title.setTextSize(TypedValue.COMPLEX_UNIT_SP, 22);
        title.setTypeface(Typeface.MONOSPACE, Typeface.BOLD);
        title.setGravity(Gravity.CENTER);
        title.setPadding(0, 0, 0, 8);
        root.addView(title);

        // Status
        statusText = new TextView(this);
        statusText.setText("Initializing...");
        statusText.setTextColor(Color.parseColor("#e0e0e0"));
        statusText.setTextSize(TypedValue.COMPLEX_UNIT_SP, 13);
        statusText.setTypeface(Typeface.MONOSPACE);
        statusText.setPadding(0, 0, 0, 8);
        root.addView(statusText);

        // Progress
        progressBar = new ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal);
        progressBar.setMax(100);
        progressBar.setProgress(0);
        progressBar.setVisibility(View.GONE);
        root.addView(progressBar);

        // Button row
        LinearLayout btnRow = new LinearLayout(this);
        btnRow.setOrientation(LinearLayout.HORIZONTAL);

        Button scanBtBtn = makeButton("SCAN BT", "#4488ff", v -> scanBluetooth());
        btnRow.addView(scanBtBtn, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1));

        Button scanOBDBtn = makeButton("SCAN OBD2", "#00ff88", v -> startOBD2Scan());
        scanOBDBtn.setTextColor(Color.BLACK);
        LinearLayout.LayoutParams p2 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1);
        p2.leftMargin = 8;
        btnRow.addView(scanOBDBtn, p2);

        Button clearBtn = makeButton("CLEAR DTC", "#ff4444", v -> clearDTCs());
        LinearLayout.LayoutParams p3 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1);
        p3.leftMargin = 8;
        btnRow.addView(clearBtn, p3);

        root.addView(btnRow);

        // Device list area (for BT scan results)
        deviceListLayout = new LinearLayout(this);
        deviceListLayout.setOrientation(LinearLayout.VERTICAL);
        deviceListLayout.setPadding(0, 8, 0, 0);
        root.addView(deviceListLayout);

        // Vehicle info
        vehicleInfoText = new TextView(this);
        vehicleInfoText.setText("");
        vehicleInfoText.setTextColor(Color.parseColor("#88ccff"));
        vehicleInfoText.setTextSize(TypedValue.COMPLEX_UNIT_SP, 12);
        vehicleInfoText.setTypeface(Typeface.MONOSPACE);
        vehicleInfoText.setPadding(0, 8, 0, 4);
        root.addView(vehicleInfoText);

        // Results scroll
        scrollView = new ScrollView(this);
        scrollView.setLayoutParams(new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, 0, 1.0f));

        dtcText = new TextView(this);
        dtcText.setText("Press SCAN BT to find your Bluetooth OBD2 adapter.\nThen press SCAN OBD2 to read vehicle data.");
        dtcText.setTextColor(Color.parseColor("#ffcc00"));
        dtcText.setTextSize(TypedValue.COMPLEX_UNIT_SP, 12);
        dtcText.setTypeface(Typeface.MONOSPACE);
        dtcText.setPadding(0, 8, 0, 8);
        scrollView.addView(dtcText);
        root.addView(scrollView);

        // Save button
        Button saveBtn = makeButton("SAVE REPORT", "#4488ff", v -> saveReport());
        LinearLayout.LayoutParams sp = new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        sp.topMargin = 8;
        saveBtn.setLayoutParams(sp);
        root.addView(saveBtn);

        setContentView(root);

        // Register BT discovery receiver
        IntentFilter filter = new IntentFilter();
        filter.addAction(BluetoothDevice.ACTION_FOUND);
        filter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        registerReceiver(btReceiver, filter);

        // Show BT status
        statusText.setText(btOBD2.getAdapterInfo());

        // Show paired devices immediately
        showPairedDevices();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try { unregisterReceiver(btReceiver); } catch (Exception e) {}
        if (btOBD2 != null) btOBD2.disconnect();
    }

    private Button makeButton(String text, String color, View.OnClickListener listener) {
        Button btn = new Button(this);
        btn.setText(text);
        btn.setBackgroundColor(Color.parseColor(color));
        btn.setTextColor(Color.WHITE);
        btn.setTypeface(Typeface.MONOSPACE, Typeface.BOLD);
        btn.setTextSize(TypedValue.COMPLEX_UNIT_SP, 11);
        btn.setPadding(8, 8, 8, 8);
        btn.setOnClickListener(listener);
        return btn;
    }

    private void showPairedDevices() {
        List<BluetoothDevice> paired = btOBD2.getPairedDevices();
        if (!paired.isEmpty()) {
            appendResult("Paired devices:\n");
            for (BluetoothDevice d : paired) {
                appendResult("  " + d.getName() + " [" + d.getAddress() + "]\n");
                addDeviceButton(d);
            }
        } else {
            appendResult("No paired Bluetooth devices.\nPress SCAN BT to discover devices.\n");
        }
    }

    private void addDeviceButton(BluetoothDevice device) {
        String label = (device.getName() != null ? device.getName() : "Unknown") +
                       " [" + device.getAddress() + "]";
        Button btn = new Button(this);
        btn.setText(label);
        btn.setBackgroundColor(Color.parseColor("#333355"));
        btn.setTextColor(Color.parseColor("#00ff88"));
        btn.setTypeface(Typeface.MONOSPACE);
        btn.setTextSize(TypedValue.COMPLEX_UNIT_SP, 10);
        btn.setPadding(8, 4, 8, 4);
        btn.setAllCaps(false);
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        lp.topMargin = 4;
        btn.setLayoutParams(lp);
        btn.setOnClickListener(v -> connectToDevice(device));
        deviceListLayout.addView(btn);
    }

    private void scanBluetooth() {
        deviceListLayout.removeAllViews();
        discoveredDevices.clear();
        statusText.setText("Scanning for Bluetooth devices...");
        dtcText.setText("");

        // Show paired devices first
        List<BluetoothDevice> paired = btOBD2.getPairedDevices();
        for (BluetoothDevice d : paired) {
            discoveredDevices.add(d);
            addDeviceButton(d);
        }

        // Start discovery for new devices
        boolean started = btOBD2.startDiscovery();
        if (started) {
            appendResult("Bluetooth discovery started...\nLooking for OBD2 adapters...\n");
        } else {
            appendResult("Could not start BT discovery.\nPaired devices shown above.\n");
        }
    }

    private void connectToDevice(BluetoothDevice device) {
        if (scanning) return;
        scanning = true;
        progressBar.setVisibility(View.VISIBLE);
        progressBar.setProgress(10);
        dtcText.setText("");

        new Thread(() -> {
            try {
                BluetoothOBD2.StatusCallback cb = msg -> handler.post(() -> statusText.setText(msg));

                boolean ok = btOBD2.connect(device, cb);
                handler.post(() -> progressBar.setProgress(30));

                if (ok) {
                    String info = btOBD2.initELM327(cb);
                    handler.post(() -> {
                        vehicleInfoText.setText("Connected: " + device.getName() + "\n" + info);
                        progressBar.setProgress(50);
                        statusText.setText("ELM327 ready! Press SCAN OBD2 for vehicle data.");
                    });
                } else {
                    handler.post(() -> {
                        appendResult("Failed to connect to " + device.getName() + "\n");
                        appendResult("Make sure the device is paired and in range.\n");
                        appendResult("Try pairing in Android Bluetooth Settings first.\n");
                    });
                }
            } catch (Exception e) {
                handler.post(() -> appendResult("Error: " + e.getMessage() + "\n"));
            } finally {
                scanning = false;
                handler.post(() -> progressBar.setVisibility(View.GONE));
            }
        }).start();
    }

    private void startOBD2Scan() {
        if (scanning) return;
        if (!btOBD2.isConnected()) {
            // Try auto-connect to first OBD2 device
            BluetoothDevice dev = btOBD2.findOBD2Device();
            if (dev != null) {
                statusText.setText("Auto-connecting to " + dev.getName() + "...");
                connectAndScan(dev);
                return;
            }
            statusText.setText("Not connected. Press SCAN BT first, then tap a device.");
            return;
        }

        scanning = true;
        progressBar.setVisibility(View.VISIBLE);
        progressBar.setProgress(0);
        dtcText.setText("");

        new Thread(() -> {
            try {
                BluetoothOBD2.StatusCallback cb = msg -> handler.post(() -> statusText.setText(msg));
                String result = btOBD2.getFullScan(cb);
                handler.post(() -> {
                    dtcText.setText(result);
                    progressBar.setProgress(100);
                    statusText.setText("Scan complete!");
                });
            } catch (Exception e) {
                handler.post(() -> appendResult("Scan error: " + e.getMessage() + "\n"));
            } finally {
                scanning = false;
                handler.post(() -> progressBar.setVisibility(View.GONE));
            }
        }).start();
    }

    private void connectAndScan(BluetoothDevice device) {
        scanning = true;
        progressBar.setVisibility(View.VISIBLE);
        dtcText.setText("");

        new Thread(() -> {
            try {
                BluetoothOBD2.StatusCallback cb = msg -> handler.post(() -> statusText.setText(msg));

                if (btOBD2.connect(device, cb)) {
                    btOBD2.initELM327(cb);
                    handler.post(() -> progressBar.setProgress(30));

                    String result = btOBD2.getFullScan(cb);
                    handler.post(() -> {
                        vehicleInfoText.setText("Connected: " + device.getName());
                        dtcText.setText(result);
                        progressBar.setProgress(100);
                        statusText.setText("Scan complete!");
                    });
                } else {
                    handler.post(() -> appendResult("Connect failed. Pair in BT settings first.\n"));
                }
            } catch (Exception e) {
                handler.post(() -> appendResult("Error: " + e.getMessage() + "\n"));
            } finally {
                scanning = false;
                handler.post(() -> progressBar.setVisibility(View.GONE));
            }
        }).start();
    }

    private void clearDTCs() {
        if (!btOBD2.isConnected()) {
            statusText.setText("Connect to OBD2 adapter first.");
            return;
        }
        if (scanning) return;
        scanning = true;

        new Thread(() -> {
            try {
                boolean ok = btOBD2.clearDTCs();
                handler.post(() -> {
                    if (ok) {
                        statusText.setText("DTCs cleared! Re-scan to verify.");
                        dtcText.setText("DTC codes cleared.\nRe-scan to verify codes are gone.\n" +
                                       "If codes return, the underlying issue persists.");
                    } else {
                        statusText.setText("Clear failed.");
                    }
                });
            } finally {
                scanning = false;
            }
        }).start();
    }

    private void saveReport() {
        try {
            String report = "OBD2 Pro Tuner - Bluetooth Scan Report\n" +
                "Generated: " + new Date() + "\n" +
                "Vehicle: 2008 Toyota FJ Cruiser\n\n" +
                vehicleInfoText.getText() + "\n\n" +
                dtcText.getText() + "\n\n" +
                "Communication Log:\n" + (btOBD2 != null ? btOBD2.getLog() : "N/A");

            File file = new File("/sdcard/Download/OBD2_BT_Report.txt");
            FileWriter writer = new FileWriter(file);
            writer.write(report);
            writer.close();

            statusText.setText("Report saved to /sdcard/Download/OBD2_BT_Report.txt");
        } catch (Exception e) {
            statusText.setText("Save failed: " + e.getMessage());
        }
    }

    private void appendResult(String text) {
        dtcText.append(text);
    }
}

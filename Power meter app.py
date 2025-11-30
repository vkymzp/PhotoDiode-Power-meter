#!/usr/bin/env python3
"""
Laser Power Meter Pro - Physics Based Calibration
-------------------------------------------------
Features:
1. Physics Formula: Power = Voltage / (Resistance * Responsivity)
2. Advanced Logging:
   - Saves Data AND Setup Info (Rate, Duration, R, R_lambda) side-by-side in columns.
   - Includes user comments in every row for easy filtering in Excel.
3. Console: Collapsible command terminal.
4. Visuals: Vector-drawn status icons.
"""

import sys
import os
import json
import csv
import time
from datetime import datetime
from collections import deque

from PyQt6 import QtWidgets, QtGui, QtCore
import pyqtgraph as pg
import serial
import serial.tools.list_ports

# --- 1. Global Plotting Configuration ---
# Use OpenGL for high-performance rendering of live data
pg.setConfigOptions(useOpenGL=True)
pg.setConfigOption('background', '#1e1e1e') # Dark background
pg.setConfigOption('foreground', '#dcdcdc') # Light text
pg.setConfigOption('antialias', True)       # Smooth lines

APP_NAME = "Laser Power Meter Pro"
CONFIG_FILE = "config_laser_pro.json"
LOGS_DIR = "logs"

# --- 2. Default Configuration ---
DEFAULT_CONFIG = {
    "calibration": {
        "resistance": 10000.0,  # 10k Ohms (Standard Op-Amp Feedback)
        "responsivity": 0.5     # 0.5 A/W (Standard Si Photodiode)
    },
    "last_connection": {
        "port": None,
        "baud": 115200
    },
    "commands": {
        "wifi_on": "CMD:WIFI_ON",
        "wifi_off": "CMD:WIFI_OFF",
        "wifi_status": "CMD:WIFI_STATUS"
    },
    "logging": {
        "last_path": "",
        "last_rate_s": 0.5,
        "last_duration_s": 60,
        "auto_start": False
    },
    "plot": {
        "window_s": 30.0,
        "lock_to_live": True
    }
}

# --- 3. Configuration Helpers ---
def load_config():
    """Loads settings from JSON or returns defaults if file missing."""
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                return json.load(f)
        except Exception:
            pass
    save_config(DEFAULT_CONFIG)
    return DEFAULT_CONFIG.copy()

def save_config(cfg):
    """Saves current settings to JSON file."""
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=2)
    except Exception:
        pass

# --- 4. Serial Worker Thread ---
class SerialWorker(QtCore.QThread):
    """
    Handles USB communication in a separate thread to keep the UI smooth.
    """
    data_received = QtCore.pyqtSignal(str)
    status_received = QtCore.pyqtSignal(str)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.alive = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.status_received.emit(f"Opened {self.port} @ {self.baud}")
        except Exception as e:
            self.status_received.emit(f"Serial open error: {e}")
            return

        while self.alive:
            try:
                if self.ser.in_waiting:
                    # Read line, decode to string, strip whitespace
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line:
                        self.data_received.emit(line)
                else:
                    self.msleep(10) # Small sleep to save CPU
            except Exception as e:
                self.status_received.emit(f"Serial read error: {e}")
                break

        if self.ser and self.ser.is_open:
            self.ser.close()
        self.status_received.emit("Serial worker stopped")

    def write(self, text: str):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write((text + "\n").encode())
                self.status_received.emit(f"Sent: {text}")
            else:
                self.status_received.emit("Serial not open")
        except Exception as e:
            self.status_received.emit(f"Serial write error: {e}")

    def stop(self):
        self.alive = False
        self.wait(1000)

# --- 5. Helper Dialogs ---
class WindowLengthDialog(QtWidgets.QDialog):
    """Dialog to set the X-axis time window."""
    def __init__(self, parent=None, default=30.0):
        super().__init__(parent)
        self.setWindowTitle("Set X Window")
        layout = QtWidgets.QFormLayout(self)
        self.len_spin = QtWidgets.QDoubleSpinBox()
        self.len_spin.setRange(0.1, 86400.0)
        self.len_spin.setValue(default)
        layout.addRow("Seconds:", self.len_spin)
        self.anchor_cb = QtWidgets.QCheckBox("Anchor to live data")
        self.anchor_cb.setChecked(True)
        layout.addRow(self.anchor_cb)
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addRow(btns)

    def get_values(self):
        return float(self.len_spin.value()), bool(self.anchor_cb.isChecked())

class AxisRangeDialog(QtWidgets.QDialog):
    """Dialog to manually set Y-axis min/max."""
    def __init__(self, axis_name, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Set {axis_name} Range")
        layout = QtWidgets.QFormLayout(self)
        self.min_e = QtWidgets.QLineEdit()
        self.max_e = QtWidgets.QLineEdit()
        layout.addRow("Min:", self.min_e)
        layout.addRow("Max:", self.max_e)
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addRow(btns)

    def get_range(self):
        try:
            return float(self.min_e.text()), float(self.max_e.text())
        except:
            return None

# --- 6. Main Application Window ---
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1200, 800)

        # Apply Modern Dark Stylesheet
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; color: #eee; }
            QLabel { color: #eee; }
            QGroupBox { border: 1px solid #555; border-radius: 5px; margin-top: 10px; font-weight: bold; color: #aaa; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox { background-color: #383838; color: #fff; border: 1px solid #555; padding: 4px; border-radius: 3px; }
            QPushButton { background-color: #444; border: 1px solid #555; padding: 6px; border-radius: 4px; color: white; }
            QPushButton:hover { background-color: #555; }
            QPlainTextEdit { background-color: #1e1e1e; color: #0f0; font-family: Consolas; }
        """)

        self.config = load_config()

        # Data Buffers (Circular buffers to hold last N points)
        self.buflen = 20000
        self.plot_x = deque(maxlen=self.buflen)
        self.plot_y = deque(maxlen=self.buflen)
        self._latest_value = None # Calculated Power in mW
        self._latest_voltage = 0.0

        # Application State
        self.serial_worker = None
        self.connected = False
        self.logging = False
        self.log_file = None
        self.log_writer = None
        self.log_start_time = None
        self.log_end_time = None
        self.console_visible = False

        # Snapshot of settings during logging
        self.logging_metadata = {}

        self.x_window_seconds = float(self.config["plot"]["window_s"])
        self.lock_to_live = self.config["plot"]["lock_to_live"]

        # Pre-render vector icons
        self.pix_on = self._create_wifi_icon(True)
        self.pix_off = self._create_wifi_icon(False)

        self._build_ui()
        self._setup_timers()

    def _create_wifi_icon(self, active):
        """Draws a vector-style WiFi icon (Green=On, Grey=Off)."""
        size = 32
        pix = QtGui.QPixmap(size, size)
        pix.fill(QtCore.Qt.GlobalColor.transparent)
        painter = QtGui.QPainter(pix)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        color = QtGui.QColor("#2ecc71") if active else QtGui.QColor("#7f8c8d")
        painter.setPen(QtGui.QPen(color, 2.5))
        cx, cy = size//2, size-4
        painter.setBrush(color)
        painter.drawEllipse(QtCore.QPoint(cx, cy), 2, 2)
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        for r in [8, 16, 22]:
            painter.drawArc(cx-r, cy-r-2, r*2, r*2, 45*16, 90*16)
        painter.end()
        return pix

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        # === LEFT COLUMN: Controls ===
        left = QtWidgets.QVBoxLayout()
        left.addWidget(QtWidgets.QLabel(f"<h2>{APP_NAME}</h2>"))

        # 1. Connection Group
        conn_box = QtWidgets.QGroupBox("Connection")
        conn_layout = QtWidgets.QFormLayout(conn_box)
        conn_box.setMaximumWidth(320)

        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports()

        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400"])
        self.baud_combo.setCurrentText(str(self.config["last_connection"]["baud"]))

        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_serial)

        conn_layout.addRow("Port:", self.port_combo)
        conn_layout.addRow("Baud:", self.baud_combo)
        conn_layout.addRow(self.btn_connect)
        self.lbl_conn_status = QtWidgets.QLabel("Disconnected")
        conn_layout.addRow(self.lbl_conn_status)
        left.addWidget(conn_box)

        # 2. Device Control Group
        ctrl_box = QtWidgets.QGroupBox("Device Control")
        ctrl_box.setMaximumWidth(320)
        ctrl_v = QtWidgets.QVBoxLayout(ctrl_box)

        # Wifi Status Row
        h_stat = QtWidgets.QHBoxLayout()
        self.icon_lbl = QtWidgets.QLabel()
        self.icon_lbl.setPixmap(self.pix_off)
        self.txt_lbl = QtWidgets.QLabel("Unknown")
        h_stat.addWidget(self.icon_lbl)
        h_stat.addWidget(self.txt_lbl)
        h_stat.addStretch()
        btn_ref = QtWidgets.QPushButton("⟳")
        btn_ref.setFixedSize(30, 30)
        btn_ref.clicked.connect(lambda: self._send_raw(self.config["commands"]["wifi_status"]))
        h_stat.addWidget(btn_ref)
        ctrl_v.addLayout(h_stat)

        # Buttons Row
        h_btn = QtWidgets.QHBoxLayout()
        btn_on = QtWidgets.QPushButton("WiFi ON")
        btn_on.setStyleSheet("background-color: #27ae60;")
        btn_on.clicked.connect(lambda: self._send_raw(self.config["commands"]["wifi_on"]))
        btn_off = QtWidgets.QPushButton("WiFi OFF")
        btn_off.setStyleSheet("background-color: #c0392b;")
        btn_off.clicked.connect(lambda: self._send_raw(self.config["commands"]["wifi_off"]))
        h_btn.addWidget(btn_on)
        h_btn.addWidget(btn_off)
        ctrl_v.addLayout(h_btn)
        left.addWidget(ctrl_box)

        # 3. Physics Calibration Group
        cal_box = QtWidgets.QGroupBox("Calibration")
        cal_box.setMaximumWidth(320)
        cal_form = QtWidgets.QFormLayout(cal_box)

        cal_cfg = self.config["calibration"]
        self.res_sb = QtWidgets.QDoubleSpinBox()
        self.res_sb.setRange(0.0, 1e9)
        self.res_sb.setValue(cal_cfg.get("resistance", 10000.0))
        self.res_sb.setSuffix(" Ω")

        self.resp_sb = QtWidgets.QDoubleSpinBox()
        self.resp_sb.setRange(0.0, 100.0)
        self.resp_sb.setValue(cal_cfg.get("responsivity", 0.5))
        self.resp_sb.setSuffix(" A/W")
        self.resp_sb.setDecimals(3)

        btn_save_cal = QtWidgets.QPushButton("Save Calibration")
        btn_save_cal.clicked.connect(self.save_calibration)

        cal_form.addRow("Resistance (R):", self.res_sb)
        cal_form.addRow("Responsivity (Rλ):", self.resp_sb)
        cal_form.addRow(QtWidgets.QLabel("P = V / (R * Rλ)"))
        cal_form.addRow(btn_save_cal)
        left.addWidget(cal_box)

        # 4. Data Logging Group
        log_box = QtWidgets.QGroupBox("Data Logging")
        log_box.setMaximumWidth(320)
        log_form = QtWidgets.QFormLayout(log_box)

        # File Browse
        file_h = QtWidgets.QHBoxLayout()
        self.file_line = QtWidgets.QLineEdit(self.config["logging"].get("last_path", ""))
        btn_browse = QtWidgets.QPushButton("...")
        btn_browse.setFixedWidth(30)
        btn_browse.clicked.connect(self.browse_log_file)
        file_h.addWidget(self.file_line)
        file_h.addWidget(btn_browse)
        log_form.addRow("File:", file_h)

        # Interval/Duration
        self.interval_sb = QtWidgets.QDoubleSpinBox()
        self.interval_sb.setRange(0.01, 3600)
        self.interval_sb.setValue(self.config["logging"].get("last_rate_s", 0.5))
        log_form.addRow("Log Rate (s):", self.interval_sb)

        self.duration_sb = QtWidgets.QSpinBox()
        self.duration_sb.setRange(1, 86400)
        self.duration_sb.setValue(self.config["logging"].get("last_duration_s", 60))
        log_form.addRow("Duration (s):", self.duration_sb)

        # Comments
        self.comments_line = QtWidgets.QLineEdit()
        self.comments_line.setPlaceholderText("e.g. Test Run 5 - Green Laser")
        log_form.addRow("Comment:", self.comments_line)

        # Auto Start
        self.auto_cb = QtWidgets.QCheckBox("Auto-start on connect")
        self.auto_cb.setChecked(self.config["logging"].get("auto_start", False))
        log_form.addRow(self.auto_cb)

        # Record Button
        self.btn_record = QtWidgets.QPushButton("Start Recording")
        self.btn_record.clicked.connect(self.toggle_recording)
        log_form.addRow(self.btn_record)

        self.lbl_log_status = QtWidgets.QLabel("Idle")
        self.lbl_log_status.setStyleSheet("color: #888; font-size: 10px;")
        log_form.addRow(self.lbl_log_status)
        left.addWidget(log_box)

        left.addStretch()
        root.addLayout(left, 1)

        # === MIDDLE COLUMN: Display & Plot ===
        mid = QtWidgets.QVBoxLayout()

        # Digital Readout
        self.lbl_val = QtWidgets.QLabel("0.000 mW")
        self.lbl_val.setFont(QtGui.QFont("Roboto Mono", 64, QtGui.QFont.Weight.Bold))
        self.lbl_val.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.lbl_val.setStyleSheet("color: #00ffcc;")
        mid.addWidget(self.lbl_val)

        # Plot Controls
        pc_h = QtWidgets.QHBoxLayout()
        self.cb_autoscale = QtWidgets.QCheckBox("Autoscale Y")
        self.cb_autoscale.setChecked(True)
        pc_h.addWidget(self.cb_autoscale)
        pc_h.addStretch()
        self.cb_lock = QtWidgets.QCheckBox("Lock to Live")
        self.cb_lock.setChecked(self.lock_to_live)
        self.cb_lock.toggled.connect(self._on_lock_changed)
        pc_h.addWidget(self.cb_lock)
        self.lbl_win = QtWidgets.QLabel(f"Window: {self.x_window_seconds}s")
        pc_h.addWidget(self.lbl_win)
        mid.addLayout(pc_h)

        # Plot Widget
        self.plot_w = pg.PlotWidget()
        self.plot_w.setBackground('#111')
        self.plot_w.showGrid(x=True, y=True, alpha=0.3)
        self.plot_w.setLabel("left", "Power", units="mW")
        self.plot_w.setLabel("bottom", "Time", units="s")
        self.curve = self.plot_w.plot(pen=pg.mkPen('#00ffcc', width=2))
        self.plot_w.scene().sigMouseClicked.connect(self._on_plot_clicked)
        mid.addWidget(self.plot_w, 1)

        # Scrollbar for history
        self.scroll = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.scroll.setRange(0, 100)
        self.scroll.setValue(100)
        mid.addWidget(self.scroll)

        # Console
        self.btn_console = QtWidgets.QPushButton("Console ▶")
        self.btn_console.setFlat(True)
        self.btn_console.setStyleSheet("text-align: left; color: #aaa;")
        self.btn_console.clicked.connect(self._toggle_console)
        mid.addWidget(self.btn_console)

        self.console_container = QtWidgets.QWidget()
        self.console_container.setVisible(False)
        c_layout = QtWidgets.QVBoxLayout(self.console_container)
        c_layout.setContentsMargins(0,0,0,0)
        self.console_text = QtWidgets.QPlainTextEdit()
        self.console_text.setReadOnly(True)
        self.console_text.setMaximumHeight(150)
        c_layout.addWidget(self.console_text)

        send_h = QtWidgets.QHBoxLayout()
        self.send_line = QtWidgets.QLineEdit()
        self.send_line.setPlaceholderText("Enter command...")
        self.send_line.returnPressed.connect(self._on_send)
        btn_send = QtWidgets.QPushButton("Send")
        btn_send.clicked.connect(self._on_send)
        send_h.addWidget(self.send_line)
        send_h.addWidget(btn_send)
        c_layout.addLayout(send_h)
        mid.addWidget(self.console_container)

        root.addLayout(mid, 3)

    def _setup_timers(self):
        self.t_plot = QtCore.QTimer()
        self.t_plot.setInterval(50)
        self.t_plot.timeout.connect(self.update_plot)
        self.t_plot.start()

        self.t_ports = QtCore.QTimer()
        self.t_ports.setInterval(2000)
        self.t_ports.timeout.connect(self.refresh_ports)
        self.t_ports.start()

    # --- Logic: Serial & Physics ---

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        cur = [self.port_combo.itemText(i) for i in range(self.port_combo.count())]
        if set(ports) != set(cur):
            self.port_combo.clear()
            self.port_combo.addItems(ports)

    def toggle_serial(self):
        if self.connected:
            if self.serial_worker: self.serial_worker.stop()
            self.serial_worker = None
            self.connected = False
            self.btn_connect.setText("Connect")
            self.lbl_conn_status.setText("Disconnected")
            self.icon_lbl.setPixmap(self.pix_off)
        else:
            p = self.port_combo.currentText()
            if not p: return
            try:
                b = int(self.baud_combo.currentText())
                self.serial_worker = SerialWorker(p, b)
                self.serial_worker.data_received.connect(self._on_data)
                self.serial_worker.status_received.connect(self._log_console)
                self.serial_worker.start()
                self.connected = True
                self.btn_connect.setText("Disconnect")
                self.lbl_conn_status.setText("Connected")

                self.config["last_connection"] = {"port": p, "baud": b}
                save_config(self.config)

                if self.auto_cb.isChecked():
                    QtCore.QTimer.singleShot(1000, self.start_logging)
            except Exception as e:
                self._log_console(str(e))

    def _on_data(self, line):
        self._log_console(f"RX: {line}")
        if "WiFi is ON" in line:
            self.icon_lbl.setPixmap(self.pix_on)
            self.txt_lbl.setText("ON")
            return
        elif "WiFi is OFF" in line:
            self.icon_lbl.setPixmap(self.pix_off)
            self.txt_lbl.setText("OFF")
            return

        try:
            # Parse Voltage
            voltage = float(line)
            self._latest_voltage = voltage

            # Physics Calculation: P = V / (R * Resp)
            R = self.res_sb.value()
            Resp = self.resp_sb.value()

            power_watts = 0.0
            if R > 0 and Resp > 0:
                power_watts = voltage / (R * Resp)

            power_mw = power_watts * 1000.0
            self._latest_value = power_mw

            self.lbl_val.setText(f"{power_mw:.4f} mW")

            ts = time.time()
            self.plot_x.append(ts)
            self.plot_y.append(power_mw)

        except ValueError:
            pass

    def update_plot(self):
        if not self.plot_x: return
        self.curve.setData(list(self.plot_x), list(self.plot_y))

        last = self.plot_x[-1]
        win = self.x_window_seconds

        if self.lock_to_live:
            self.plot_w.setXRange(last - win, last)
            self.scroll.setValue(100)

        if self.cb_autoscale.isChecked():
             self.plot_w.enableAutoRange(axis=pg.ViewBox.YAxis)

    def _on_lock_changed(self, state):
        self.lock_to_live = state
        self.config["plot"]["lock_to_live"] = state
        save_config(self.config)

    def _on_plot_clicked(self, ev):
        if ev.button() == QtCore.Qt.MouseButton.RightButton:
            menu = QtWidgets.QMenu()
            act_x = menu.addAction("Set X Window...")
            act_y = menu.addAction("Set Y Range...")
            res = menu.exec(QtGui.QCursor.pos())

            if res == act_x:
                dlg = WindowLengthDialog(self, self.x_window_seconds)
                if dlg.exec():
                    val, anchor = dlg.get_values()
                    self.x_window_seconds = val
                    self.lbl_win.setText(f"Window: {val}s")
                    self.cb_lock.setChecked(anchor)
            elif res == act_y:
                dlg = AxisRangeDialog("Y", self)
                if dlg.exec():
                    rng = dlg.get_range()
                    if rng:
                        self.cb_autoscale.setChecked(False)
                        self.plot_w.setYRange(rng[0], rng[1])

    def save_calibration(self):
        self.config["calibration"]["resistance"] = self.res_sb.value()
        self.config["calibration"]["responsivity"] = self.resp_sb.value()
        save_config(self.config)
        self._log_console("Calibration Saved.")

    # --- Logging Logic ---
    def browse_log_file(self):
        os.makedirs(LOGS_DIR, exist_ok=True)
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Log", os.path.join(LOGS_DIR, "log.csv"), "CSV (*.csv)")
        if path:
            self.file_line.setText(path)
            self.config["logging"]["last_path"] = path
            save_config(self.config)

    def toggle_recording(self):
        if not self.logging:
            self.start_logging()
        else:
            self.stop_logging()

    def start_logging(self):
        """Starts logging data to CSV with Metadata columns alongside data."""
        path = self.file_line.text()
        if not path:
            self.browse_log_file()
            path = self.file_line.text()
            if not path: return

        try:
            # 1. Capture Current Settings (Snapshot)
            self.logging_metadata = {
                "comment": self.comments_line.text(),
                "rate": self.interval_sb.value(),
                "duration": self.duration_sb.value(),
                "res": self.res_sb.value(),
                "resp": self.resp_sb.value()
            }

            # 2. Open File and Write Extended Header
            self.log_file = open(path, "w", newline='')
            self.log_writer = csv.writer(self.log_file)

            # --- HEADER DEFINITION ---
            # Columns 1-4: Dynamic Data
            # Columns 5-9: Static Metadata (Setup Info)
            header = [
                "Timestamp_ISO",
                "Unix_Time",
                "Raw_Voltage_V",
                "Power_mW",
                "Comment",
                "Interval_Setting_s",
                "Duration_Setting_s",
                "Resistance_Used_Ohms",
                "Responsivity_Used_AW"
            ]
            self.log_writer.writerow(header)

            # 3. Initialize State
            self.logging = True
            self.log_start_time = time.time()
            self.log_end_time = self.log_start_time + self.logging_metadata["duration"]

            self.btn_record.setText("Stop Recording")
            self.btn_record.setStyleSheet("background-color: #c0392b;")
            self.lbl_log_status.setText(f"Recording to {os.path.basename(path)}")

            # 4. Start Log Timer
            self.t_log = QtCore.QTimer()
            self.t_log.timeout.connect(self._log_tick)
            self.t_log.start(int(self.logging_metadata["rate"] * 1000))

            self._log_console(f"Logging started: {path}")

        except Exception as e:
            self._log_console(f"Log start error: {e}")

    def _log_tick(self):
        """Writes one row of data + metadata to the CSV."""
        if not self.logging: return
        if time.time() > self.log_end_time:
            self.stop_logging()
            return

        if self._latest_value is not None:
            try:
                # Construct the Row
                row = [
                    datetime.now().isoformat(),       # Timestamp
                    time.time(),                      # Unix
                    self._latest_voltage,             # Voltage
                    self._latest_value,               # Power
                    # --- Metadata Columns (Repeated every row) ---
                    self.logging_metadata["comment"],
                    self.logging_metadata["rate"],
                    self.logging_metadata["duration"],
                    self.logging_metadata["res"],
                    self.logging_metadata["resp"]
                ]
                self.log_writer.writerow(row)
                self.log_file.flush()
            except:
                pass

    def stop_logging(self):
        self.logging = False
        if hasattr(self, 't_log'): self.t_log.stop()
        if self.log_file: self.log_file.close()
        self.btn_record.setText("Start Recording")
        self.btn_record.setStyleSheet("")
        self.lbl_log_status.setText("Idle")
        self._log_console("Logging stopped.")

    # --- Console Logic ---
    def _toggle_console(self):
        self.console_visible = not self.console_visible
        self.console_container.setVisible(self.console_visible)
        self.btn_console.setText("Console ▼" if self.console_visible else "Console ▶")

    def _on_send(self):
        txt = self.send_line.text()
        if txt:
            self._send_raw(txt)
            self.send_line.clear()

    def _send_raw(self, txt):
        if self.connected and self.serial_worker:
            self.serial_worker.write(txt)
        else:
            self._log_console("Not connected.")

    def _log_console(self, txt):
        ts = datetime.now().strftime("%H:%M:%S")
        self.console_text.appendPlainText(f"[{ts}] {txt}")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

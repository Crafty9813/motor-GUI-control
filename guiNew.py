import struct
import sys
import time
import threading
from collections import deque
from contextlib import contextmanager
from serial import Serial

import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QDoubleSpinBox, QGroupBox,
    QFormLayout, QComboBox
)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject

import mit_func


# ── Configurable motor parameters (index, name, unit, default) ────────────────
# Indices with bit 0x80 set are integer-valued; the rest are IEEE floats.
CONFIG_PARAMS = [
    (0, "Pole pairs", "", 21.0),
    (1, "Gear ratio", "", 18.0),
    (2, "Torque constant", "N·m/A", 2.97 / 18.0),
    (3, "Electrical zero offset", "rad", 0.0),
    (4, "Mechanical zero offset", "rad", 0.0),
    (5, "Current limit", "A", 60.0),
    (6, "Position min", "rad", -12.57),
    (7, "Position max", "rad", 12.57),
    (8, "Velocity min", "rad/s", -65.0),
    (9, "Velocity max", "rad/s", 65.0),
    (10, "Max position gain", "kP", 500.0),
    (11, "Max velocity gain", "kD", 5.0),
    (12, "Calibration current", "A", 10.0),
    (13, "D/Q position gain", "kp on d/q axes", 0.05),
    (14, "D/Q integral gain", "ki on d/q axes", 200.0),
    (15, "Temp min", "°C", -40.0),
    (16, "Temp max", "°C", 180.0),
    (17, "Resistance", "Ω", 0.05),
    (18, "Inductance", "H", 0.00001),
    (0x80 | 0, "Encoder select", "0=internal,1=external", 0),
    (0x80 | 1, "Phase order", "0 same order as encoder, 1 reversed", 0),
    (0x80 | 3, "CAN master", "", 0),
    (0x80 | 4, "CAN timeout", "ms", 10000),
    (0x80 | 5, "Calibration done", "", 0),
]

# ── Live plot channels ────────────────────────────────────────────────────────
# `key` order must match the tuple returned by mit_func.decode_reply().
# si_prefix=False keeps axes in base units (e.g. "rad" instead of "mrad").
PLOT_CHANNELS = [
    {"key": "position", "title": "Position (rad)",   "label": "Position",    "units": "rad",   "color": "cyan",    "row": 0, "col": 0, "colspan": 1, "si_prefix": False},
    {"key": "velocity", "title": "Velocity (rad/s)", "label": "Velocity",    "units": "rad/s", "color": "lime",    "row": 0, "col": 1, "colspan": 1, "si_prefix": False},
    {"key": "torque",   "title": "Torque (N-m)",     "label": "Torque",      "units": "N-m",   "color": "yellow",  "row": 1, "col": 0, "colspan": 1, "si_prefix": True},
    {"key": "vbus",     "title": "Bus Voltage (V)",  "label": "Voltage",     "units": "V",     "color": "magenta", "row": 1, "col": 1, "colspan": 1, "si_prefix": True},
    {"key": "temp",     "title": "Temperature (°C)", "label": "Temperature", "units": "°C",    "color": "red",     "row": 2, "col": 0, "colspan": 2, "si_prefix": True},
]
REPLY_FIELDS = tuple(ch["key"] for ch in PLOT_CHANNELS)


class DataEmitter(QObject):
    """Bridges the serial reader thread to the GUI thread (Qt signals are
    thread-safe; direct widget access from a worker thread is not)."""
    data_received = pyqtSignal(dict)


class MotorControlGUI(QMainWindow):
    def __init__(self, port="/dev/ttyUSB0", baudrate=921600):
        super().__init__()
        self.setWindowTitle("Motor Control GUI")
        self.setGeometry(100, 100, 1600, 900)

        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.serial_lock = threading.Lock()
        # When set, the background reader stops consuming so a config
        # transaction (read/write parameter) can capture its own reply
        # instead of it being swallowed by the plotting path.
        self._config_busy = threading.Event()

        self.emitter = DataEmitter()
        self.emitter.data_received.connect(self.on_data_received)

        self.config_defaults = {index: value for index, _, _, value in CONFIG_PARAMS}

        # Rolling history for the plots (shared timestamps + one buffer/channel).
        self.max_points = 1000
        self.timestamps = deque(maxlen=self.max_points)
        self.buffers = {ch["key"]: deque(maxlen=self.max_points) for ch in PLOT_CHANNELS}
        self.start_time = None

        self.init_ui()
        self.connect_serial()
        self.start_read_thread()

    # ── UI construction ───────────────────────────────────────────────────────
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        left_panel = QVBoxLayout()
        left_panel.addWidget(self._build_control_group())
        left_panel.addWidget(self._build_mode_group())
        left_panel.addWidget(self._build_param_group())

        self.send_btn = QPushButton("SEND COMMAND")
        self.send_btn.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 10px;")
        self.send_btn.clicked.connect(self.send_command)
        left_panel.addWidget(self.send_btn)

        left_panel.addWidget(self._build_status_group())
        left_panel.addStretch()

        right_panel = QVBoxLayout()
        right_panel.addWidget(self._build_plots())

        main_layout = QHBoxLayout()
        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 2)
        central_widget.setLayout(main_layout)

        # Refresh the plots at a fixed rate, decoupled from the serial rate.
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(100)  # ms

        # Periodic command sending (poll / control loop); started in MIT mode.
        self.auto_send_timer = QTimer()
        self.auto_send_timer.timeout.connect(self.auto_send_command)
        self.auto_send_timer.setInterval(5)  # ms

    def _build_control_group(self):
        group = QGroupBox("Motor Control")
        layout = QFormLayout()

        self.can_id_input = QLineEdit("1200FD01")
        layout.addRow("CAN ID (8 hex):", self.can_id_input)

        self.position_input = self._make_spinbox(mit_func.P_MIN, mit_func.P_MAX, 0.0, 0.1)
        layout.addRow("Position (rad):", self.position_input)

        self.velocity_input = self._make_spinbox(mit_func.V_MIN, mit_func.V_MAX, 0.0, 1.0)
        layout.addRow("Velocity (rad/s):", self.velocity_input)

        self.kp_input = self._make_spinbox(0, mit_func.KP_MAX, 20.0, 1.0)
        layout.addRow("KP (N-m/rad):", self.kp_input)

        self.kd_input = self._make_spinbox(0, mit_func.KD_MAX, 0.5, 0.1)
        layout.addRow("KD (N-m·s/rad):", self.kd_input)

        self.torque_input = self._make_spinbox(mit_func.T_MIN, mit_func.T_MAX, 0.0, 0.5)
        layout.addRow("Torque FF (N-m):", self.torque_input)

        group.setLayout(layout)
        return group

    @staticmethod
    def _make_spinbox(minimum, maximum, value, step):
        box = QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setValue(value)
        box.setSingleStep(step)
        return box

    def _build_mode_group(self):
        group = QGroupBox("Mode")
        layout = QVBoxLayout()

        self.mit_mode_btn = QPushButton("Enable MIT Mode")
        self.mit_mode_btn.clicked.connect(self.set_mit_mode)
        layout.addWidget(self.mit_mode_btn)

        self.calibration_mode_btn = QPushButton("Enable Calibration Mode")
        self.calibration_mode_btn.clicked.connect(self.set_calibration_mode)
        layout.addWidget(self.calibration_mode_btn)

        self.menu_mode_btn = QPushButton("Disable (MENU Mode)")
        self.menu_mode_btn.clicked.connect(self.set_menu_mode)
        layout.addWidget(self.menu_mode_btn)

        group.setLayout(layout)
        return group

    def _build_param_group(self):
        group = QGroupBox("Parameter Tuning")
        layout = QFormLayout()

        self.param_can_id_input = QLineEdit("00000001")
        self.param_can_id_input.setPlaceholderText("8-digit hex")
        layout.addRow("Config CAN ID:", self.param_can_id_input)

        self.param_combo = QComboBox()
        for index, name, unit, _ in CONFIG_PARAMS:
            self.param_combo.addItem(f"{name} ({unit})" if unit else name, index)
        self.param_combo.currentIndexChanged.connect(self.on_param_selection_changed)
        layout.addRow("Parameter:", self.param_combo)

        self.param_value_input = QLineEdit("0.0")
        layout.addRow("Value:", self.param_value_input)

        buttons = QHBoxLayout()
        self.read_param_btn = QPushButton("Read")
        self.read_param_btn.clicked.connect(self.read_selected_parameter)
        buttons.addWidget(self.read_param_btn)

        self.write_param_btn = QPushButton("Write")
        self.write_param_btn.clicked.connect(self.write_selected_parameter)
        buttons.addWidget(self.write_param_btn)

        self.reset_params_btn = QPushButton("Reset Defaults")
        self.reset_params_btn.clicked.connect(self.reset_parameters)
        buttons.addWidget(self.reset_params_btn)
        layout.addRow(buttons)

        self.param_status_label = QLabel("No config action yet")
        layout.addRow(self.param_status_label)

        group.setLayout(layout)
        self.on_param_selection_changed()
        return group

    def _build_status_group(self):
        group = QGroupBox("Status")
        layout = QVBoxLayout()
        self.status_label = QLabel("Disconnected")
        layout.addWidget(self.status_label)
        group.setLayout(layout)
        return group

    def _build_plots(self):
        pg.setConfigOption('background', '#1a1a1a')
        pg.setConfigOption('foreground', '#ffffff')

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.curves = {}
        for ch in PLOT_CHANNELS:
            plot = self.plot_widget.addPlot(
                title=ch["title"], row=ch["row"], col=ch["col"], colspan=ch["colspan"]
            )
            plot.setLabel('bottom', 'Time', units='s')
            plot.setLabel('left', ch["label"], units=ch["units"])
            if not ch["si_prefix"]:
                plot.getAxis('left').enableAutoSIPrefix(False)
            self.curves[ch["key"]] = plot.plot(pen=pg.mkPen(ch["color"], width=2))
        return self.plot_widget

    # ── Serial helpers ────────────────────────────────────────────────────────
    def connect_serial(self):
        try:
            self.serial = Serial(self.port, self.baudrate, timeout=0.1)
            self.status_label.setText(f"Connected to {self.port}")
            self.running = True
        except Exception as e:
            self.status_label.setText(f"Failed to connect: {e}")
            self.running = False

    @staticmethod
    def _hex_bytes(data_bytes):
        return " ".join(f"{b:02X}" for b in data_bytes)

    def _clear_serial_buffer(self):
        if not self.serial:
            return
        try:
            with self.serial_lock:
                if self.serial.in_waiting:
                    self.serial.read(self.serial.in_waiting)
        except Exception:
            pass

    def _extract_can_payload(self, raw_data):
        if raw_data is None or len(raw_data) <= 9:
            return None
        return raw_data[7:-2]

    def _send_can_frame(self, can_id, data_bytes, timeout=0.25):
        if not self.serial:
            raise ConnectionError("Serial port is not connected")

        serial_cmd = mit_func.can2serial(can_id, self._hex_bytes(data_bytes))
        deadline = time.time() + timeout
        buffer = b""

        with self.serial_lock:
            self.serial.write(serial_cmd)
            time.sleep(0.05)

        while time.time() < deadline:
            with self.serial_lock:
                n = self.serial.in_waiting
                if n:
                    buffer += self.serial.read(n)
                    payload = self._extract_can_payload(buffer)
                    if payload and len(payload) >= 8:
                        return payload
            time.sleep(0.1)

        return None

    def _send_mode_frame(self, mode):
        if not self.serial:
            raise ConnectionError("Serial port is not connected")

        mode_data = f"FF FF FF FF FF FF FF {mode:02X}"
        mode_serial = mit_func.can2serial("00000001", mode_data)
        with self.serial_lock:
            self.serial.write(mode_serial)
            time.sleep(0.05)
            if self.serial.in_waiting:
                self.serial.read(self.serial.in_waiting)  # Clear buffer

    # ── Parameter (config) transactions ───────────────────────────────────────
    @contextmanager
    def _config_transaction(self):
        """Pause the background reader for the duration of a config
        read/write so its reply reaches _send_can_frame instead of being
        consumed and mis-decoded by the plotting path."""
        self._config_busy.set()
        try:
            # Let the reader notice the flag and finish any in-flight read,
            # then clear stale bytes so we only see our own reply.
            time.sleep(0.02)
            self._clear_serial_buffer()
            self._send_mode_frame(mit_func.MENU_MODE)
            yield
        finally:
            self._config_busy.clear()

    @staticmethod
    def _format_param_value(index, value):
        """Integer params (bit 0x80) render as ints; floats trim trailing zeros."""
        if index & 0x80:
            return str(int(value))
        return f"{value:.6f}".rstrip("0").rstrip(".")

    def _param_name(self):
        return self.param_combo.currentText().split(" (")[0]

    def on_param_selection_changed(self):
        param_index = int(self.param_combo.currentData())
        default_value = self.config_defaults.get(param_index, 0.0)
        self.param_value_input.setText(self._format_param_value(param_index, default_value))

    def read_selected_parameter(self):
        try:
            param_index = int(self.param_combo.currentData())
            payload = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, param_index]
            print(f"Reading parameter index {param_index} with payload: {payload}")
            with self._config_transaction():
                reply = self._send_can_frame(self.param_can_id_input.text(), payload, timeout=0.2)
            print(f"Reply: {reply}")
            if reply is None:
                self.param_status_label.setText("No reply for parameter read")
                return

            value = struct.unpack(">f", bytes(reply[2:6]))[0]
            self.param_value_input.setText(self._format_param_value(param_index, value))
            self.param_status_label.setText(f"Read {self._param_name()} = {value}")
        except Exception as e:
            self.param_status_label.setText(f"Config read error: {e}")

    def write_selected_parameter(self):
        try:
            param_index = int(self.param_combo.currentData())
            value = float(self.param_value_input.text())
            payload = [0xFF, 0xFF, 0xFD, param_index] + list(struct.pack(">f", value))
            with self._config_transaction():
                reply = self._send_can_frame(self.param_can_id_input.text(), payload, timeout=0.2)
            if reply is None:
                self.param_status_label.setText("No reply for parameter write")
                return

            written_value = struct.unpack(">f", bytes(reply[2:6]))[0]
            self.param_status_label.setText(f"Wrote {self._param_name()} = {written_value}")
        except Exception as e:
            self.param_status_label.setText(f"Config write error: {e}")

    def reset_parameters(self):
        try:
            payload = [0xFF, 0xFF, 0xFD, 0xFF, 0x00, 0x00, 0x00, 0x00]
            with self._config_transaction():
                reply = self._send_can_frame(self.param_can_id_input.text(), payload, timeout=0.2)
            if reply is None:
                self.param_status_label.setText("Reset command sent, no confirmation reply")
            else:
                self.param_status_label.setText("Parameters reset to defaults")
        except Exception as e:
            self.param_status_label.setText(f"Reset error: {e}")

    # ── Background reader & plotting ──────────────────────────────────────────
    def start_read_thread(self):
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    def read_serial_loop(self):
        if not self.serial:
            return

        while self.running:
            try:
                # A config transaction owns the port; don't steal its reply.
                if self._config_busy.is_set():
                    time.sleep(0.005)
                    continue

                with self.serial_lock:
                    n = self.serial.in_waiting
                    if not n:
                        time.sleep(0.01)
                        continue
                    raw_data = self.serial.read(n)

                if raw_data and len(raw_data) > 9:
                    can_data = self._extract_can_payload(raw_data)
                    if can_data and len(can_data) >= 8:
                        result = mit_func.decode_reply(can_data)
                        if result:
                            if self.start_time is None:
                                self.start_time = time.time()
                            data = dict(zip(REPLY_FIELDS, result))
                            data['time'] = time.time() - self.start_time
                            self.emitter.data_received.emit(data)
                time.sleep(0.1)
            except Exception as e:
                print(f"Read error: {e}")

    def on_data_received(self, data):
        self.timestamps.append(data['time'])
        for key, buffer in self.buffers.items():
            buffer.append(data[key])

    def update_plots(self):
        if not self.timestamps:
            return
        t = list(self.timestamps)
        for key, curve in self.curves.items():
            curve.setData(t, list(self.buffers[key]))

    # ── Command sending ───────────────────────────────────────────────────────
    def _current_command_frame(self):
        """Build the serial frame for the current control inputs."""
        command_bytes = mit_func.pack_cmd(
            p=self.position_input.value(),
            v=self.velocity_input.value(),
            kp=self.kp_input.value(),
            kd=self.kd_input.value(),
            t_ff=self.torque_input.value(),
        )
        return mit_func.can2serial(self.can_id_input.text(), self._hex_bytes(command_bytes))

    def send_command(self):
        try:
            serial_cmd = self._current_command_frame()
            if self.serial:
                with self.serial_lock:
                    self.serial.write(serial_cmd)
            self.status_label.setText(
                f"Command sent: p={self.position_input.value():.2f}, "
                f"kp={self.kp_input.value():.1f}, kd={self.kd_input.value():.2f}"
            )
        except Exception as e:
            self.status_label.setText(f"Error: {e}")

    def auto_send_command(self):
        """Send the current command periodically to poll the motor / run the
        control loop. This elicits regular status replies for plotting and
        keeps the controller fed."""
        try:
            if not self.serial:
                return
            serial_cmd = self._current_command_frame()
            with self.serial_lock:
                self.serial.write(serial_cmd)
        except Exception:
            pass

    # ── Mode control ──────────────────────────────────────────────────────────
    def set_mit_mode(self):
        try:
            self._send_mode_frame(mit_func.MIT_MODE)
            self.status_label.setText("MIT Mode enabled")
            # Start periodic auto-send so we get regular replies for plotting.
            self.auto_send_timer.start()
        except Exception as e:
            self.status_label.setText(f"Error: {e}")

    def set_calibration_mode(self):
        try:
            self._send_mode_frame(mit_func.CALIBRATION_MODE)
            self.status_label.setText("Calibration Mode enabled")
        except Exception as e:
            self.status_label.setText(f"Error: {e}")

    def set_menu_mode(self):
        try:
            self._send_mode_frame(mit_func.MENU_MODE)
            self.status_label.setText("MENU Mode enabled (motor disabled)")
            # Stop periodic auto-send when motor is disabled.
            self.auto_send_timer.stop()
        except Exception as e:
            self.status_label.setText(f"Error: {e}")

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def closeEvent(self, event):
        self.running = False
        if self.serial:
            self.serial.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = MotorControlGUI()
    gui.show()
    sys.exit(app.exec_())

import sys
import time
import threading
from collections import deque
from serial import Serial

import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QSpinBox, QDoubleSpinBox, QGroupBox,
    QFormLayout, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject

import mit_func


# Signal emitter for thread-safe GUI updates
class DataEmitter(QObject):
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
        self.emitter = DataEmitter()
        self.emitter.data_received.connect(self.on_data_received)

        # Data buffers (max 1000 points)
        self.max_points = 1000
        self.timestamps = deque(maxlen=self.max_points)
        self.positions = deque(maxlen=self.max_points)
        self.velocities = deque(maxlen=self.max_points)
        self.torques = deque(maxlen=self.max_points)
        self.vbus = deque(maxlen=self.max_points)
        self.temps = deque(maxlen=self.max_points)
        self.start_time = None

        self.init_ui()
        self.connect_serial()
        self.start_read_thread()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout()

        # Left panel: controls
        left_panel = QVBoxLayout()

        # Control group
        control_group = QGroupBox("Motor Control")
        control_layout = QFormLayout()

        # CAN ID
        self.can_id_input = QLineEdit("1200FD01")
        control_layout.addRow("CAN ID (8 hex):", self.can_id_input)

        # Position
        self.position_input = QDoubleSpinBox()
        self.position_input.setRange(mit_func.P_MIN, mit_func.P_MAX)
        self.position_input.setValue(0.0)
        self.position_input.setSingleStep(0.1)
        control_layout.addRow("Position (rad):", self.position_input)

        # Velocity
        self.velocity_input = QDoubleSpinBox()
        self.velocity_input.setRange(mit_func.V_MIN, mit_func.V_MAX)
        self.velocity_input.setValue(0.0)
        self.velocity_input.setSingleStep(1.0)
        control_layout.addRow("Velocity (rad/s):", self.velocity_input)

        # KP
        self.kp_input = QDoubleSpinBox()
        self.kp_input.setRange(0, mit_func.KP_MAX)
        self.kp_input.setValue(20.0)
        self.kp_input.setSingleStep(1.0)
        control_layout.addRow("KP (N-m/rad):", self.kp_input)

        # KD
        self.kd_input = QDoubleSpinBox()
        self.kd_input.setRange(0, mit_func.KD_MAX)
        self.kd_input.setValue(0.5)
        self.kd_input.setSingleStep(0.1)
        control_layout.addRow("KD (N-m·s/rad):", self.kd_input)

        # Torque (t_ff)
        self.torque_input = QDoubleSpinBox()
        self.torque_input.setRange(mit_func.T_MIN, mit_func.T_MAX)
        self.torque_input.setValue(0.0)
        self.torque_input.setSingleStep(0.5)
        control_layout.addRow("Torque FF (N-m):", self.torque_input)

        control_group.setLayout(control_layout)
        left_panel.addWidget(control_group)

        # Mode group
        mode_group = QGroupBox("Mode")
        mode_layout = QVBoxLayout()

        self.mit_mode_btn = QPushButton("Enable MIT Mode")
        self.mit_mode_btn.clicked.connect(self.set_mit_mode)
        mode_layout.addWidget(self.mit_mode_btn)

        self.menu_mode_btn = QPushButton("Disable (MENU Mode)")
        self.menu_mode_btn.clicked.connect(self.set_menu_mode)
        mode_layout.addWidget(self.menu_mode_btn)

        mode_group.setLayout(mode_layout)
        left_panel.addWidget(mode_group)

        # Send button
        self.send_btn = QPushButton("SEND COMMAND")
        self.send_btn.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 10px;")
        self.send_btn.clicked.connect(self.send_command)
        left_panel.addWidget(self.send_btn)

        # Status
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        self.status_label = QLabel("Disconnected")
        status_layout.addWidget(self.status_label)
        status_group.setLayout(status_layout)
        left_panel.addWidget(status_group)

        left_panel.addStretch()

        # Right panel: plots
        right_panel = QVBoxLayout()

        # Enable dark background
        pg.setConfigOption('background', '#1a1a1a')
        pg.setConfigOption('foreground', '#ffffff')

        # Create plot widget
        self.plot_widget = pg.GraphicsLayoutWidget()
        right_panel.addWidget(self.plot_widget)

        # Position plot
        self.pos_plot = self.plot_widget.addPlot(title="Position (rad)", row=0, col=0)
        self.pos_plot.setLabel('bottom', 'Time', units='s')
        self.pos_plot.setLabel('left', 'Position', units='rad')
        self.pos_curve = self.pos_plot.plot(pen=pg.mkPen('cyan', width=2))

        # Velocity plot
        self.vel_plot = self.plot_widget.addPlot(title="Velocity (rad/s)", row=0, col=1)
        self.vel_plot.setLabel('bottom', 'Time', units='s')
        self.vel_plot.setLabel('left', 'Velocity', units='rad/s')
        self.vel_curve = self.vel_plot.plot(pen=pg.mkPen('lime', width=2))

        # Torque plot
        self.torque_plot = self.plot_widget.addPlot(title="Torque (N-m)", row=1, col=0)
        self.torque_plot.setLabel('bottom', 'Time', units='s')
        self.torque_plot.setLabel('left', 'Torque', units='N-m')
        self.torque_curve = self.torque_plot.plot(pen=pg.mkPen('yellow', width=2))

        # Vbus plot
        self.vbus_plot = self.plot_widget.addPlot(title="Bus Voltage (V)", row=1, col=1)
        self.vbus_plot.setLabel('bottom', 'Time', units='s')
        self.vbus_plot.setLabel('left', 'Voltage', units='V')
        self.vbus_curve = self.vbus_plot.plot(pen=pg.mkPen('magenta', width=2))

        # Temperature plot
        self.temp_plot = self.plot_widget.addPlot(title="Temperature (°C)", row=2, col=0, colspan=2)
        self.temp_plot.setLabel('bottom', 'Time', units='s')
        self.temp_plot.setLabel('left', 'Temperature', units='°C')
        self.temp_curve = self.temp_plot.plot(pen=pg.mkPen('red', width=2))

        # Combine layouts
        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 2)

        central_widget.setLayout(main_layout)

        # Timer for plot updates
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(100)  # Update every 100ms

    def connect_serial(self):
        try:
            self.serial = Serial(self.port, self.baudrate, timeout=0.1)
            self.status_label.setText(f"Connected to {self.port}")
            self.running = True
        except Exception as e:
            self.status_label.setText(f"Failed to connect: {e}")
            self.running = False

    def start_read_thread(self):
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    def read_serial_loop(self):
        if not self.serial:
            return

        while self.running:
            try:
                raw_data = self.serial.read(self.serial.in_waiting)
                if raw_data and len(raw_data) > 9:
                    can_data = raw_data[7:-2]
                    if len(can_data) >= 8:
                        result = mit_func.decode_reply(can_data)
                        if result:
                            position, velocity, torque, vbus, temp = result
                            if self.start_time is None:
                                self.start_time = time.time()
                            elapsed = time.time() - self.start_time

                            self.emitter.data_received.emit({
                                'time': elapsed,
                                'position': position,
                                'velocity': velocity,
                                'torque': torque,
                                'vbus': vbus,
                                'temp': temp
                            })
                time.sleep(0.01)
            except Exception as e:
                print(f"Read error: {e}")

    def on_data_received(self, data):
        self.timestamps.append(data['time'])
        self.positions.append(data['position'])
        self.velocities.append(data['velocity'])
        self.torques.append(data['torque'])
        self.vbus.append(data['vbus'])
        self.temps.append(data['temp'])

    def update_plots(self):
        if len(self.timestamps) > 0:
            t = list(self.timestamps)
            self.pos_curve.setData(t, list(self.positions))
            self.vel_curve.setData(t, list(self.velocities))
            self.torque_curve.setData(t, list(self.torques))
            self.vbus_curve.setData(t, list(self.vbus))
            self.temp_curve.setData(t, list(self.temps))

    def send_command(self):
        try:
            can_id = self.can_id_input.text()
            position = self.position_input.value()
            velocity = self.velocity_input.value()
            kp = self.kp_input.value()
            kd = self.kd_input.value()
            t_ff = self.torque_input.value()

            command_bytes = mit_func.pack_cmd(p=position, v=velocity, kp=kp, kd=kd, t_ff=t_ff)
            data_hex = " ".join(f"{b:02X}" for b in command_bytes)
            serial_cmd = mit_func.can2serial(can_id, data_hex)

            self.serial.write(serial_cmd)
            self.status_label.setText(f"Command sent: p={position:.2f}, kp={kp:.1f}, kd={kd:.2f}")
        except Exception as e:
            self.status_label.setText(f"Error: {e}")

    def set_mit_mode(self):
        try:
            mode = mit_func.MIT_MODE
            mode_data = f"FF FF FF FF FF FF FF {mode:02X}"
            mode_serial = mit_func.can2serial("00000001", mode_data)
            self.serial.write(mode_serial)
            self.status_label.setText("MIT Mode enabled")
            time.sleep(0.05)
            self.serial.read(self.serial.in_waiting)  # clear buffer
        except Exception as e:
            self.status_label.setText(f"Error: {e}")

    def set_menu_mode(self):
        try:
            mode = mit_func.MENU_MODE
            mode_data = f"FF FF FF FF FF FF FF {mode:02X}"
            mode_serial = mit_func.can2serial("00000001", mode_data)
            self.serial.write(mode_serial)
            self.status_label.setText("MENU Mode enabled (motor disabled)")
            time.sleep(0.05)
            self.serial.read(self.serial.in_waiting)  # clear buffer
        except Exception as e:
            self.status_label.setText(f"Error: {e}")

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

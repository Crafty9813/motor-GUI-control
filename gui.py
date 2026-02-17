"""
Simple GUI using Tkinter that shows the encoder, velocity, and torque data as graphs
Can control the motor by sending desired velocities, angles (in rad), and torque via serial.
"""

import serial
import threading
import tkinter as tk
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import math

PORT = '/dev/ttyACM0'
BAUD = 115200
MAX_POINTS = 300

ser = serial.Serial(PORT, BAUD, timeout=1)

angle_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
vel_data   = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
torque_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

def serial_reader():
    while ser.is_open:
        try:
            line = ser.readline().decode().strip()

            if not line.startswith("A,"):
                continue

            parts = line.split(",")

            if len(parts) != 4:
                continue

            angle = float(parts[1])
            vel = float(parts[2])
            torque = float(parts[3])

            angle_data.append(angle)
            vel_data.append(vel)
            torque_data.append(torque)

        except Exception as e:
            print("Serial error:", e)

threading.Thread(target=serial_reader, daemon=True).start()

# GUI using Tkinter
root = tk.Tk()
root.title("Motor Control GUI")

# Matplotlib
fig, ax = plt.subplots(3, 1, figsize=(12, 10))

line_angle, = ax[0].plot(angle_data)
ax[0].set_title("Angle")
ax[0].set_ylim(0, 2*math.pi)

line_vel, = ax[1].plot(vel_data)
ax[1].set_title("Velocity")
ax[1].set_ylim(0, 30)

line_torque, = ax[2].plot(torque_data)
ax[2].set_title("Torque")
ax[2].set_ylim(0, 10)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

def update_plot(frame):
    line_angle.set_ydata(angle_data)
    line_vel.set_ydata(vel_data)
    return line_angle, line_vel

ani = animation.FuncAnimation(fig, update_plot, interval=50)

# COMMAND FUNCS
def send_velocity():
    ser.write(b'm')
    value=vel_slider.get()
    cmd = f"v{value}\r"
    ser.write(cmd.encode())

def send_position():
    ser.write(b'm')
    value = pos_entry.get()
    cmd = f"p{value}\r"
    ser.write(cmd.encode())

def send_torque():
    ser.write(b'm')
    value = torque_entry.get()
    cmd = f"q{value}\r"
    ser.write(cmd.encode())

def stop_motor():
    ser.write(b'\x1b') # esc key

# tk controls
control_frame = tk.Frame(root)
control_frame.pack()

tk.Label(control_frame, text="Velocity", font=("Arial", 13, "bold")).grid(row=0, column=0)
vel_slider = tk.Scale(control_frame, from_=0, to=30, orient=tk.HORIZONTAL)
vel_slider.grid(row=0, column=1, columnspan=2, sticky="we")
tk.Button(control_frame, bg='lightblue', text="Set", command=send_velocity).grid(row=0, column=3)

tk.Label(control_frame, text="Position", font=("Arial", 13, "bold")).grid(row=1, column=0)
pos_entry = tk.Entry(control_frame)
pos_entry.grid(row=1, column=1)
tk.Button(control_frame, bg='lightblue', text="Move", command=send_position).grid(row=1, column=3)

tk.Label(control_frame, text="Torque (I_Q_des)", font=("Arial", 13, "bold")).grid(row=2, column=0)
torque_entry = tk.Entry(control_frame)
torque_entry.grid(row=2, column=1)
tk.Button(control_frame, bg='lightblue', text="Set", command=send_torque).grid(row=2, column=3)

tk.Button(control_frame, text="STOP", bg="red", command=stop_motor)\
    .grid(row=3, column=0, columnspan=3)

root.mainloop()

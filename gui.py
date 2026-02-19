"""
Simple GUI using Tkinter that shows encoder, velocity, and torque data as graphs and allows Kp (position) and Kd (velocity) tuning
Can control the motor by sending desired velocities, angles (in rad), and desired torque via serial.
"""

import serial
import threading
import tkinter as tk
#from tkinter import ttk
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import math

PORT = '/dev/ttyACM0'
BAUD = 115200
MAX_POINTS = 300

ser = serial.Serial(PORT, BAUD, timeout=1)
ser.write(b'm')

angle_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
vel_data   = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
torque_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
#kp_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS) TODO: maybe get Kp and Kd data and put it on something to viz

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
            #kp = float(parts[4])
            #kd = float(parts[5])

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
ax[2].set_ylim(0, 30)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

def update_plot(frame):
    line_angle.set_ydata(angle_data)
    line_vel.set_ydata(vel_data)
    line_torque.set_ydata(torque_data)
    return line_angle, line_vel, line_torque

ani = animation.FuncAnimation(fig, update_plot, interval=50)

# COMMAND FUNCS
def send_velocity():
    #ser.write(b'm')
    value=vel_slider.get()
    cmd = f"v{value}\r"
    ser.write(cmd.encode())

def send_position():
    #ser.write(b'm')
    value = pos_entry.get()
    cmd = f"p{value}\r"
    ser.write(cmd.encode())

def send_torque():
    #ser.write(b'm')
    value = torque_entry.get()
    cmd = f"q{value}\r"
    ser.write(cmd.encode())

def send_kp():
    #ser.write(b'm')
    value = kp_entry.get()
    cmd = f"u{value}\r"
    ser.write(cmd.encode())

def send_kd():
    #ser.write(b'm')
    value = kd_entry.get()
    cmd = f"d{value}\r"
    ser.write(cmd.encode())

def stop_motor():
    #ser.write(b'\x1b') # esc key
    cmd = f"v0\r"
    ser.write(cmd.encode())

# tk controls
control_frame = tk.Frame(root)
control_frame.pack()

tk.Label(control_frame, text="Velocity", font=("Arial", 21, "bold")).grid(row=0, column=0, sticky="news")
vel_slider = tk.Scale(control_frame, font=("Arial", 12, "bold"), from_=0, to=30, orient=tk.HORIZONTAL, width=25)
vel_slider.grid(row=0, column=1, columnspan=2, sticky="we")
tk.Button(control_frame, bg='lightblue', text="Set", font= ("Times", 18, "bold"), command=send_velocity).grid(row=0, column=3)

tk.Label(control_frame, text="Position", font=("Arial", 21, "bold")).grid(row=1, column=0)
pos_entry = tk.Entry(control_frame, font=("Times New Roman", 18))
pos_entry.grid(row=1, column=1)
tk.Button(control_frame, bg='lightblue', text="Move", font= ("Times", 18, "bold"), command=send_position).grid(row=1, column=3)

tk.Label(control_frame, text="Torque (t_ff)", font=("Arial", 21, "bold")).grid(row=2, column=0)
torque_entry = tk.Entry(control_frame, font=("Times New Roman", 18))
torque_entry.grid(row=2, column=1)
tk.Button(control_frame, bg='lightblue', text="Set", font= ("Times", 18, "bold"), command=send_torque).grid(row=2, column=3)

tk.Label(control_frame, text="Kp (0 init)", font=("Arial", 21, "bold")).grid(row=3, column=0)
kp_entry = tk.Entry(control_frame, font=("Times New Roman", 18))
kp_entry.grid(row=3, column=1)
tk.Button(control_frame, bg='lightblue', text="Set", font= ("Times", 18, "bold"), command=send_kp).grid(row=3, column=3)

tk.Label(control_frame, text="Kd (0 init)", font=("Arial", 21, "bold")).grid(row=4, column=0)
kd_entry = tk.Entry(control_frame, font=("Times New Roman", 18))
kd_entry.grid(row=4, column=1)
tk.Button(control_frame, bg='lightblue', text="Set", font= ("Times", 18, "bold"), command=send_kd).grid(row=4, column=3)

tk.Button(control_frame, text="STOP", bg="red", font= ("Times", 18, "bold"), command=stop_motor).grid(row=5, column=1)

root.mainloop()

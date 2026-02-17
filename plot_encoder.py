"""
Plots encoder angle data
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import math

PORT = '/dev/ttyACM0'
BAUD = 115200
MAX_POINTS = 200

# Serial comm
ser = serial.Serial(PORT, BAUD, timeout=1)

# Data buffer
data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

# Plot setup
fig, ax = plt.subplots()
line, = ax.plot(data)
ax.set_ylim(0, 2*math.pi)
ax.set_title("Live Encoder Angle")
ax.set_ylabel("Angle (rad)")

def update(frame):
    try:
        line_data = ser.readline().decode().strip()
        
        if "angle:" in line_data:
            value = float(line_data.split(":")[1])
            data.append(value)
            line.set_ydata(data)
    except:
        pass

    return line,

ani = animation.FuncAnimation(fig, update, interval=50)
plt.show()

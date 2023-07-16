import serial
import time
import serial.tools.list_ports as port_list
import numpy as np


def generate_g_code(rot, feed, motor_axis):
    rot_code = bytes(str(rot), "ASCII")
    feed_code = bytes(str(feed), "ASCII")
    return b'G1 ' + bytes(str(motor_axis), "ASCII") + rot_code + b' F' + feed_code + b'\r\n'


def zero_axes(serial):
    serial.write(generate_g_code(0, 600, "X"))
    serial.write(generate_g_code(0, 600, "Y"))
    serial.write(generate_g_code(0, 600, "Z"))


# Find all COM ports on computer
ports = list(port_list.comports())

for p in ports:
    print(p)

# select correct COM Port
port = "COM8"
s = serial.Serial(port, 115200)

input("Enter to begin test...")

# Waiting for serial port startup
s.write(b'\r\n\r\n')
time.sleep(2)
s.flushInput()

# G Code to be sent

# G1 Z10 F600
t = 0
dt = 0.01

z_split = np.arange(0, -110, step=-2.5)
maneuver_time = 7.5
x_go_to = -92.5

dir = 0

for i in z_split:
    print(f"Setting Z to: {i}")
    s.write(generate_g_code(i, 500, "Z"))
    dir += 1
    if dir % 2 != 0:
        s.write(generate_g_code(x_go_to, 750, "X"))
        print(f"Setting X to: {x_go_to}")
    else:
        s.write(generate_g_code(0, 750, "X"))
        print(f"Setting X to: {0}")

    time.sleep(maneuver_time)


input("Done with test! Setting home...")
zero_axes(s)

# print("Sending: ",x)

# simple write to send G Code

# s.write(x1)
# s.write(x0)

# Close serial port to end

s.close()

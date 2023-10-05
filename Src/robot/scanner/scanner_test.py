'''
    REF: https://www.ramcoi.com/customer/docs/skudocs/optex_cd-22_rs485_manual.pdf

    TODO:
        - Position management
        - Stepper motor interpolation
        - Display points in Open3D
'''
import math

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from serial import Serial, PARITY_NONE, EIGHTBITS

PORT = "COM3"
BAUD = 9600

CMD_READ = 0x52
CMD_WRITE = 0x57
CMD_FUNC = 0x43

STX = 0x02
ETX = 0x03


class LaserScanner():

    def __init__(self, port, baud):
        self.laser = Serial(port=port, baudrate=baud, bytesize=EIGHTBITS, parity=PARITY_NONE)

        scanner.tx_rx(0x52, 0x40, 0x06, True)  # generate test packet

    def read_distance(self):
        # begin read sequence by handshaking with laser TODO
        reply = self.tx_rx(CMD_WRITE, 0x00, 0x01)
        data_1, data_2 = self.decode_response(reply)

        # combine data 1 and data 2 into a single var
        distance_mm = (data_1 << 8) | data_2

        # correct for signed-ness
        if distance_mm & 0x8000: distance_mm = distance_mm - 0x10000
        distance_mm += 5000  # add 5000 to make laser range 0-10000 (0-100mm)

        return distance_mm

    def tx_rx(self, command, data_1, data_2, verbose=False):
        # generate bit checksum
        bcc = hex(command ^ data_1 ^ data_2)
        if verbose: print("calculated BCC: ", bcc)

        # construct packet and send out serially
        data_packet = bytes(hex(STX) + hex(command) + hex(data_1) + hex(data_2) + hex(ETX) + bcc, 'ASCII')
        if verbose: print("sending out: ", data_packet)
        self.laser.write(data_packet)

        # wait for reply
        reply = self.laser.read_until(ETX)

        return reply

    def decode_response(self, packet):
        packet_str = packet.decode('ASCII')

        # Extract components from the packet
        stx = int(packet_str[0:2], 16)
        command = int(packet_str[2:4], 16)
        data_1 = int(packet_str[4:6], 16)
        data_2 = int(packet_str[6:8], 16)
        etx = int(packet_str[8:10], 16)
        bcc = int(packet_str[10:], 16)

        # Verify start and end characters
        if stx != STX or etx != ETX:
            raise ValueError("[LASER] Invalid packet: Start or end character mismatch")

        # Verify no errors were raised
        if data_1 == 0x02:
            raise ValueError("[LASER] Command address is invalid!")
        elif data_1 == 0x04:
            raise ValueError("[LASER] Command BCC is invalid!")
        elif data_1 == 0x05:
            raise ValueError("[LASER] Unknown command provided!")
        elif data_1 == 0x06 or data_1 == 0x07:
            raise ValueError("[LASER] Failed to set parameter, out of range!")

        # Verify the checksum
        if bcc != (command ^ data_1 ^ data_2):
            raise ValueError("[LASER] Invalid packet: Checksum mismatch")

        return data_1, data_2

    def shutdown(self):
        return
        self.laser.close()

    def __del__(self):
        return
        self.shutdown()


'''

    Test plotting of live sensor data using matplotlib

'''

REFRESH_RATE = 120  # Hz
DT = 1 / REFRESH_RATE

MAX_X = 200

# define new figure
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = list(range(0, MAX_X))  # x-axis samples
ys = [0] * MAX_X  # y-axis data, ie. sensor data
ax.set_ylim([0, 150])

# create line to update
line, = ax.plot(xs, ys)

# set labels
plt.title("CD-22 Sensor Data")
plt.ylabel("Distance (mm)")
plt.xlabel("Sample #")

# setup device
scanner = LaserScanner("COM3", 9600)
scanner.tx_rx(0x52, 0x40, 0x06)


def update_plot(i, ys):
    data = scanner.read_distance()

    # append new time and data
    ys.append(data)

    # limit lists to 20 items
    ys = ys[-MAX_X:]

    # update line
    line.set_ydata(ys)

    return line,


ani = animation.FuncAnimation(fig, update_plot, fargs=(xs,), interval=(DT * math.pow(10, 3)), blit=True)
plt.show()

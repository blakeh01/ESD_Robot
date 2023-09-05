import numpy as np
import time

from serial import Serial, PARITY_NONE, EIGHTBITS

from Src.robot.arm.stepperhandler import StepperHandler

## CONSTS

PORT = "COM3"
BAUD = 9600

CMD_READ = 0x52
CMD_WRITE = 0x57
CMD_FUNC = 0x43

STX = 0x02
ETX = 0x03

class ScanPathing:

    def __init__(self, StepperHandler: stepper_handler):
        self.stepper_handler = stepper_handler
        self.LDS = LDS(PORT, BAUD)

        self.xc = np.array([])
        self.yc = np.array([])
        self.zc = np.array([])
        self.ca = np.array([])

        # todo configurable?
        self.x_start = 0
        self.z_start = 0
        self.x_end = 203
        self.z_end = 203
        self.y_start = 0
        self.y_end = 203

        self.step = 1
        self.big_step = 10

        self.degree = 45

        self.flag = True  # this needs the value of the LDS read into it then converted based on the displacement to origin

        self.rotations = 360 / self.degree

    def begin_scan(self):
        for h in range(0, int(self.rotations)):

            # move_servo_r(r_start)
            # move_servo_th(th_start)
            # move_servo_z(z_start)

            cur_axis = h * self.degree

            for k in range(self.y_start, self.y_end):  # Calculate next path between points
                y_pos = k

                for j in range(self.z_start, self.z_end):
                    z_pos = j

                    for i in range(self.x_start, self.x_end):
                        x_pos = i

                        if flag:
                            xc = np.append(self.xc, [x_pos])
                            # zc = np.append(yc, [z_pos])
                            # yc = np.append(zc, [y_pos]) # change this to the adjusted read in value from the sensor
                            # ca = np.append(ca, [cur_axis])
                        if x_pos == self.x_end - 1:
                            pass
                            # move_servo_x(x_start)
                            # time.sleep(0.00001)  # Wait for motor to reset to beginning of loop
                        else:
                            pass
                            # move_servo_x(x_pos + step)
                            # time.sleep(.00001)  # change this to just longer than feed rate
                    if z_pos == self.z_end - 1:
                        pass
                        # move_servo_z(z_start)
                        # time.sleep(0.00001)  # reset to beginning of loop
                    else:
                        pass
                        # print('not z end')
                        # move_servo_z(z_pos + step)
                        # time.sleep(.00001)  # change this to just longer than feed rate
                # move_servo_y(y_pos + big_step)
                # time.sleep(1)
            # time.sleep(1)
            # rotate center platform by degree
        # return the xc, yc, zc, and ca to point verification

class LDS:

    def __init__(self, port, baud):
        self.laser = Serial(port=port, baudrate=baud, bytesize=EIGHTBITS, parity=PARITY_NONE)

        scanner.tx_rx(0x52, 0x40, 0x06, True) # generate test packet

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
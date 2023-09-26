import serial
import time

## TODO: CONFIG?
LDS_PORT = "COM7"
LDS_BAUD = 115200

CMD_READ    = 0x52
CMD_WRITE   = 0x57
CMD_FUNC    = 0x43

STX         = 0x02
ETX         = 0x03

class SerialMonitor:

    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None

        self.open_connection()

    def open_connection(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate)
            print(f"Connected to {self.port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            print(f"Error: {e}")

    def close_connection(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Connection closed.")
        else:
            print("No active connection to close.")

    def write_data(self, data):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(data)
        else:
            print("No active connection to write to.")

    def read_data(self):
        if self.serial_conn and self.serial_conn.is_open:
            return self.serial_conn.readline().decode().strip()
        else:
            print("No active connection to read from.")

class StepperHandler(SerialMonitor):

    def __init__(self, port, baud):
        super(StepperHandler, self).__init__(port, baud)

        if not self.serial_conn:
            input("Failed to connect to DUET Stepper Controller, please ensure the board is powered and all connections are present!")
            quit()

        self.initialized = False
        self.initialize_motor()

    def initialize_motor(self):
        self.serial_conn.write(b'\r\n\r\n')
        time.sleep(1)
        self.serial_conn.flushInput()
        print(f"[DUET] Stepper motors enabled!")
        print("[DUET] Current position is home.")
        self.initialized = True

    def write_x(self, pos, feed=100):
        code = bytes(str(f"G1 X{round(pos)} F{round(feed)}"), "ASCII")
        self.serial_conn.flushInput()
        self.serial_conn.write(code + b'\r\n')
        print(code)

    def write_y(self, pos, feed=100):
        code = bytes(str(f"G1 Y{round(pos)} F{round(feed)}"), "ASCII")
        self.serial_conn.flushInput()
        self.serial_conn.write(code + b'\r\n')
        print(code)

    def write_z(self, pos, feed=100):
        code = bytes(str(f"G1 Z{round(pos)} F{round(feed)}"), "ASCII")
        self.serial_conn.flushInput()
        self.serial_conn.write(code + b'\r\n')
        print(code)

    def write_a(self, pos, feed=100):
        code = bytes(str(f"G1 A{round(pos)} F{round(feed)}"), "ASCII")
        self.serial_conn.flushInput()
        self.serial_conn.write(code + b'\r\n')
        print(code)

    def write_b(self, pos, feed=100):
        code = bytes(str(f"G1 B{round(pos)} F{round(feed)}"), "ASCII")
        self.serial_conn.flushInput()
        self.serial_conn.write(code + b'\r\n')
        print(code)

    def close(self):
        print(f"[DUET] Exiting... going home.")
        print("[DUET] I did nothing... please implement me :]")
        self.stepper_serial.close()

class LDS:

    def __init__(self, port=LDS_PORT, baud=LDS_BAUD):
        try:
            self.laser = serial.Serial(port=port, baudrate=baud, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE)
        except:
            input("Could not connect to LDS, please check all connections and ensure the laser is on!")
            quit()

        # read in the name to confirm connection
        _, data_2 = self.decode_response(self.tx_rx(CMD_READ, 0x01, 0x00))
        if data_2 == 100: print("Connection successful! Model: CD22-100-485")

    def read_distance(self):
        # begin read sequence by handshaking with laser TODO
        reply = self.tx_rx(CMD_FUNC, 0xB0, 0x01)
        data_1, data_2 = self.decode_response(reply)

        # combine data 1 and data 2 into a single var
        distance_mm = (data_1 << 8) | data_2

        # correct for signed-ness
        if distance_mm & 0x8000:
            distance_mm -= 0x10000

        # clamp
        # if distance_mm < -5000 or distance_mm > 5000:
        #     distance_mm = 0
        # else:
        #     distance_mm += 5000

        return distance_mm

    def tx_rx(self, command, data_1, data_2, packet_size=6, verbose=False):
        self.laser.flush()

        bcc = command ^ data_1 ^ data_2
        if verbose: print("Generated BCC: ", bcc)

        packet = bytearray([STX, command, data_1, data_2, ETX, bcc])
        if verbose: print("Generated Packet: ", packet)

        self.laser.write(packet)

        # wait for reply, return it
        reply = self.laser.read(size=packet_size)

        return reply

    def decode_response(self, packet):
        stx = packet[0]
        command = packet[1]
        data_1 = packet[2]
        data_2 = packet[3]
        etx = packet[4]
        bcc = packet[5]

        # Verify start and end characters
        if stx != STX or etx != ETX:
            raise ValueError("[LASER] Invalid packet: Start or end character mismatch")

        # Verify no errors were raised
        if command == 0x15:  # 'NAK'
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
import time

import board
import busio
import digitalio
import pwmio
from adafruit_motor import servo

# create a PWMOut object on Pin A2.
pwm = pwmio.PWMOut(board.D5, duty_cycle=2 ** 15, frequency=50)
relay = digitalio.DigitalInOut(board.D6)
uart = busio.UART(board.TX, board.RX, baudrate=9600)

# Create a servo object, my_servo.
my_servo = servo.Servo(pwm)

while True:
    if uart.in_waiting:
        data = uart.read(1).decode()  # Read one byte and decode it to a string
        if data == '1':
            relay = relay.value = True
            my_servo.angle = 180
            time.sleep(2)
            relay.value = False
            my_servo = 0
            time.sleep(1)
            uart.write("ok\n")  # may be unused.

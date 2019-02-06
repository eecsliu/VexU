import serial
import time

ser = serial.Serial('/dev/ttyS0')
ser.timeout = 1

def send_heartbeat():
    ser.write(b'+++')

def enable_motor():
    ser.write('n'.charAt(0))

def disable_motor():
    ser.write('f'.charAt(0))

while True:
    send_heartbeat()
    time.sleep(1)
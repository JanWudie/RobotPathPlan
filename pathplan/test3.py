import serial
import time

ser = serial.Serial('/dev/cu.usbmodem14101', 9600, timeout=0.1)
print(ser.name)
ser.reset_input_buffer()

import serial
import time


ser = serial.Serial('/dev/cu.usbmodem14201', 9600, timeout=0.1)
print(ser.name)
ser.reset_input_buffer()

instrList = ["DD 00 00", "DD 00 00", "DD 00 00", "DD 00 00", "DD 00 00", "DD 00 00"]
for item in instrList:
    ser.write(item.encode("utf-8"))
    line = ser.readline().decode('utf-8').rstrip()
    print(line)
ser.close()


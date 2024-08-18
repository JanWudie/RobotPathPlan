import serial
import time


ser = serial.Serial('/dev/cu.usbmodem14101', 9600, timeout=0.1)
print(ser.name)
ser.reset_input_buffer()

time.sleep(2)
instrList = ["Nonsense", "BB 99 50\n","BB 99 50\n","BB 99 50\n","BB 99 50\n"]
for item in instrList:
    ser.write(item.encode("utf-8"))
    #time.sleep(1)
    #print(line)
    if(item == "BB 99 50\n"):
        time.sleep(3)
    data = ser.readline().decode('utf-8').rstrip()
    """
    while data != "0":
    #while ser.in_waiting:  # Or: while ser.inWaiting():
        #print("fuck you")
        data = ser.readline().decode('utf-8').rstrip()
    """
    print(data)
    print(item)
ser.close()

#"AA 99 00\n", "BB 99 50\n", "CC 99 50\n", "BB 99 50\n", "CC 99 50\n", "DD 00 00\n"

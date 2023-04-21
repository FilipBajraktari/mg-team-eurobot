import time
import serial
import numpy as np
import serial.tools.list_ports


# Init Pyserial
ports = serial.tools.list_ports.comports()
ser = serial.Serial()
portList = []



for p in ports:
    portList.append(str(p))
    print(str(p))


val = input("select port: /dev/ttyUSB")
#val = 1

for x in range(0, len(portList)):
    if portList[x].startswith("/dev/ttyUSB" + str(val)):
        portVar = "/dev/ttyUSB" + str(val)

ser.baudrate = 115200
ser.port = portVar

ser.open()

while True:
    command = input("Type in a command: ")
    if(command == "close"):
        ser.close()
        break
    ser.write(bytearray(command, 'ascii'))
    time.sleep(0.1)
    if ser.inWaiting():
        s = ""
        while ser.inWaiting():
            s += ser.read().decode('ascii')
        print(s)

import csv
import time
import serial
import numpy as np
import serial.tools.list_ports

# Import data from existing csv file
def import_data(file_path):
    data = []
    with open(file_path, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            # data.append(int(row[0]))
            data.append(float(row[0]))
    
    return np.array(data)


# Init Pyserial
def init_serial(port_val=3, baud_rate=2000000):
    ports = serial.tools.list_ports.comports()
    ser = serial.Serial()
    portList = []

    for p in ports:
        portList.append(str(p))
        print(str(p))

    val = input("select port: COM")
    #val = port_val # Filip 3   Kosta 7

    for x in range(0, len(portList)):
        if portList[x].startswith("COM" + str(val)):
            portVar = "COM" + str(val)

    ser.baudrate = baud_rate #2200000
    ser.port = portVar

    return ser

# Read data from Serial
def read_data(ser, writing_path, number_of_samples=5000):
    f = open(writing_path, 'w', newline='')
    csvwriter = csv.DictWriter(f, fieldnames=['pos'])

    countdown = 3
    for i in range(countdown, 0, -1):
        print(str(i))
        time.sleep(1)
    print('0')
    ser.open()

    i = 0
    while True:
        if ser.inWaiting():
            s = ser.readline().decode('utf').rstrip('\r\n')
            break
    while i < number_of_samples:
        if ser.inWaiting():
            s = ser.readline().decode('utf').rstrip('\r\n')
            csvwriter.writerow({'pos':float(s)})
            # print(s)
            i += 1
    f.close()

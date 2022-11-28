import serial.tools.list_ports
import time
import csv

ports = serial.tools.list_ports.comports()
ser = serial.Serial()
portList = []

log = True

for p in ports:
    portList.append(str(p))
    print(str(p))

# val = input("select port: COM")
val = 3 # Filip
# val = 7 # Kosta

for x in range(0, len(portList)):
    if portList[x].startswith("COM" + str(val)):
        portVar = "COM" + str(val)

ser.baudrate = 2200000
ser.port = portVar

f = open('testing.csv', 'w', newline='')
csvwriter = csv.DictWriter(f, fieldnames=['pos'])

if (log == True):
    print('3')
    time.sleep(1)
    print('2')
    time.sleep(1)
    print('1')
    time.sleep(1)
    print('0')
    ser.open()

    i = 0
    while(i < 5000):
        if ser.inWaiting():
            s = ser.readline().decode('utf').rstrip('\r\n')
            csvwriter.writerow({'pos':int(s)})
            #print(s)
            i+=1
    f.close()
    print("log done, continuing in print mode")
    while True:
        if ser.inWaiting():
            s = ser.readline().decode('utf').rstrip('\r\n')
            print(s)
else:
    ser.open()
    while True:
        if ser.inWaiting():
            s = ser.readline().decode('utf').rstrip('\n')
            print(s)
            

  

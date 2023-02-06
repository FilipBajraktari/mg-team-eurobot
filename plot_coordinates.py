import sys
import os
import matplotlib.pyplot as plt

home = os.path.dirname(os.path.realpath("wheel_encoder.py"))

sys.path.append(os.path.abspath(home + "/simulation"))
from simulation.import_data import *

def main():
    ser = init_serial()

    number_of_samples = 15000
    writing_path = "coordinates.csv"
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
            s = ser.readline().decode('utf').rstrip('\n')
            break
    while i < number_of_samples:
        if ser.inWaiting():
            s = ser.readline().decode('utf').rstrip('\n')
            # RobotX, RobotY, RobotRot = s.split(";")
            # csvwriter.writerow({'pos':float(s)})
            # print(RobotX, "   ", RobotY, "   ", RobotRot)
            print(s.split(";"))
            i += 1
    f.close()

if __name__=="__main__":
    main()
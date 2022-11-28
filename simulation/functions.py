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
            data.append(int(row[0]))
    
    return np.array(data)


# Init Pyserial
def init_serial(port_val=3, baud_rate=2200000):
    ports = serial.tools.list_ports.comports()
    ser = serial.Serial()
    portList = []

    for p in ports:
        portList.append(str(p))
        print(str(p))

    # val = input("select port: COM")
    val = port_val # Filip 3   Kosta 7

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

    print('3')
    time.sleep(1)
    print('2')
    time.sleep(1)
    print('1')
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
            csvwriter.writerow({'pos':int(s)})
            i += 1
    f.close()



def simple_differentiator(system_input, sample_time):
    output = [0]
    for i in range(1, len(system_input)):
        derivative = (system_input[i] - system_input[i-1]) / sample_time
        output.append(derivative)

    return np.array(output)

'''
def example():
    # Duration of the simulation in sec
    time = 10

    # Define system
    omega = 5
    sample_time = 0.005
    number_of_iterations = int(time / sample_time)
    # differentiator = System([omega, 0], [1, omega], sample_time)
    differentiator = Differentiator(omega, sample_time)

    # Define system input response
    # system_input = np.arange(number_of_iterations)
    system_input = np.sin(np.arange(0, time, sample_time))
    # system_input = np.ones(number_of_iterations)

    # System output
    system_output =  differentiator.inpute_response(system_input, number_of_iterations)

    # Plot the system response
    plt.plot(np.arange(0, time, sample_time), system_output)
    plt.plot(np.arange(0, time, sample_time), simple_differentiator(system_input, sample_time))
    plt.grid()
    plt.show()
'''
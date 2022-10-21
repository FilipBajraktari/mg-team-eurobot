import csv
import numpy as np

def import_data(file_path):
    data = []
    with open(file_path, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            data.append(int(row[0]))
    
    return np.array(data)

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
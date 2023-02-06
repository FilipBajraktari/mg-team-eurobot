import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution

home = os.path.dirname(os.path.realpath("wheel_encoder.py"))

sys.path.append(os.path.abspath(home + "/simulation"))
from system import *
from functions import *
from import_data import *


parser = argparse.ArgumentParser()
parser.add_argument("-st", "--Sample_time", type=float)
parser.add_argument("-tf", "--Testing_file", type=str)
args = parser.parse_args()

default_sample_time = 0.001
sample_time = args.Sample_time if args.Sample_time else default_sample_time
TESTING_FILE = True if args.Testing_file != None and \
                       args.Testing_file.lower() == "true" else False


# Function that convert encoder counts to radians
def counts_to_radians(data, counts_per_revolution=8000):
    return data * (2 * np.pi / counts_per_revolution)

def estimated_path(angular_velocity, dt):
    angular_distance = [0]
    for i, omega in enumerate(angular_velocity):
        angular_distance.append(angular_distance[i] + omega*dt)

    return np.array(angular_distance)

def error(data, estimated_data):
    return np.sum(np.square(data - estimated_data))

def optimal_omega(data, sample_time):
    def error_function(ulaz):
        omega, sample_time = ulaz
        differentiator = Differentiator(omega, sample_time)

        estimated_angular_velocity = differentiator.inpute_response(data)
        estimated_angular_distance = estimated_path(estimated_angular_velocity, sample_time)

        return np.sum(np.square(estimated_angular_distance[1:] - data))

    omega_min = 5
    omega_max = 200
    omega_range = [(omega_min, omega_max), (sample_time, sample_time)]
    optimized_omega = differential_evolution(error_function, bounds=omega_range)

    print(optimized_omega, end="\n\n")
    return optimized_omega.x[0]


def main():

    if TESTING_FILE:
        ser = init_serial()
        read_data(ser, writing_path = os.path.abspath(home + "/encoder_data/testing.csv"), number_of_samples=25000)
        data_file = home + "/encoder_data/testing.csv"
    else:    
        data_file = home + "/encoder_data/10000hz.csv"

    # data = counts_to_radians(import_data(data_file))
    data = import_data(data_file)

    # Differentiator
    omega = 81
    # omega = optimal_omega(data, sample_time)
    differentiator = Differentiator(omega, sample_time)
    differentiator.params()

    angular_velocity = differentiator.inpute_response(data)
    angular_distance = estimated_path(angular_velocity, sample_time)


    # Plot The System Response
    what_to_plot = 2
    time = sample_time * len(data)
    if what_to_plot == 1:
        plt.plot(np.arange(0, time, sample_time), angular_velocity, label="advanced differentiator")
        plt.plot(np.arange(0, time, sample_time), simple_differentiator(data, sample_time), label="simple differentiator")
    elif what_to_plot == 2:
        plt.plot(np.arange(0, time, sample_time), data, label="encoder value")
        # plt.plot(np.arange(0, time, sample_time), angular_distance[1:], label="estimated encoder value")
        # plt.plot(np.arange(0, time, sample_time), angular_distance[1:] - data, label="estimated encoder value")

    plt.legend(loc="upper right")
    plt.grid()
    plt.show()

if __name__=="__main__":
    main()
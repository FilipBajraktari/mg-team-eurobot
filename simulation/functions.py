import numpy as np


def simple_differentiator(system_input, sample_time):
    output = [0]
    for i in range(1, len(system_input)):
        derivative = (system_input[i] - system_input[i-1]) / sample_time
        output.append(derivative)

    return np.array(output)
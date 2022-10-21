import control
import numpy as np
import matplotlib.pyplot as plt

class System:
    def __init__(self, num, den, dt):
        self.num = np.array(num)
        self.den = np.array(den)
        self.sys_tf = control.tf(num, den)
        self.sys_discrete_tf = control.sample_system(self.sys_tf, dt, method='zoh')
        self.sys_ss = control.tf2ss(self.sys_discrete_tf)
        self.x1 = np.zeros(self.sys_ss.B.shape)

    def next_output(self, sys_input):
        x = self.sys_ss.A.dot(self.x1) + self.sys_ss.B*sys_input
        y = self.sys_ss.C.dot(self.x1) + self.sys_ss.D*sys_input
        self.x1 = x

        return y

    def inpute_response(self, system_input):
        output = []
        number_of_iterations = len(system_input)
        for i in range(number_of_iterations):
            y = self.next_output(system_input[i])
            output.append(y[0][0])

        return output

class Differentiator(System):
    def __init__(self, omega, sample_time):
        super().__init__([omega, 0], [1, omega], sample_time)
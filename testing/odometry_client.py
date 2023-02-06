import dbus

import matplotlib.pyplot as plt
from canvas_testing import custom_canvas

import random
from datetime import datetime

def main():
    bus = dbus.SessionBus()
 
    remote_object = bus.get_object("com.mgrobotics.OdometryService",
                                   "/StateSpace")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")

    fig, ax = custom_canvas()
    while True:
        current_state_space = iface.get_random_state_space()
        x_coordinate = current_state_space[0]
        y_coordinate = current_state_space[1]
        theta        = current_state_space[2]
        print(*current_state_space, end='\t')

        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        print(current_time)

        ax.scatter(x_coordinate, y_coordinate, s=3, c='blue')
        plt.draw()
        plt.pause(random.uniform(0, 5))

if __name__ == '__main__':
    main()
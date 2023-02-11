import dbus
import dbus.service
import dbus.mainloop.glib

import matplotlib.pyplot as plt
from canvas_testing import custom_canvas

import random
from datetime import datetime

def AI_signal_handler(new_desired_position):
    print("AI has just told me to go to:\t", ' '.join(new_desired_position))

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()
 
    remote_object = bus.get_object("com.mgrobotics.Service",
                                   "/StateSpace")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")

    # Define signals that should be received
    bus.add_signal_receiver(AI_signal_handler)

    fig, ax = custom_canvas()
    while True:
        # current_state_space = iface.get_random_state_space()
        current_state_space = iface.get_state_space()
        x_coordinate = current_state_space[0]
        y_coordinate = current_state_space[1]
        theta        = current_state_space[2]
        print(*current_state_space, end='\t')

        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        print(current_time)

        ax.scatter(x_coordinate, y_coordinate, s=3, c='blue')
        plt.draw()
        # plt.pause(random.uniform(0, 5))
        plt.pause(0.1)

if __name__ == '__main__':
    main()
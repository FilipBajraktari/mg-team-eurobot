import dbus
import dbus.service
import dbus.mainloop.glib

import matplotlib.pyplot as plt
from canvas_testing import custom_canvas

import random
from datetime import datetime

# DESIRED POSITION COORDINATES
x_coordinate = None
y_coordinate = None
theta = None

def catchall_new_desired_position(desired_position):
    global x_coordinate, y_coordinate, theta
    x_coordinate, y_coordinate, theta = desired_position

def rrt_star():
    pass

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()
 
    remote_object = bus.get_object("com.mgrobotics.Service",
                                   "/StateSpace")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    iface_pp = dbus.Interface(remote_object, "com.mgrobotics.PurePursuit")

    fig, ax = custom_canvas()
    while True:
        current_state_space = iface.get_random_state_space()
        # current_state_space = iface.get_state_space()
        x_coordinate = current_state_space[0]
        y_coordinate = current_state_space[1]
        theta        = current_state_space[2]
        print(*current_state_space, end='\t')

        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        print(current_time)

        # Send a lookup position
        iface_pp.emit_new_lookahead([x_coordinate, y_coordinate, theta])

        ax.scatter(x_coordinate, y_coordinate, s=3, c='blue')
        plt.draw()
        # plt.pause(random.uniform(0, 5))
        plt.pause(0.1)

if __name__ == '__main__':
    main()
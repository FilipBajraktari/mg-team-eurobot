from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

import math
import random

class StateSpace(dbus.service.Object):

    @dbus.service.method("com.mgrobotics.OdometryInterface", 
                         in_signature='', out_signature='ad')
    def get_state_space(self):
        return []
    
    @dbus.service.method("com.mgrobotics.OdometryInterface", 
                         in_signature='', out_signature='ad')
    def get_random_state_space(self):
        x_coordinate = random.uniform(-150, 150)
        y_coordinate = random.uniform(-100, 100)
        theta = random.uniform(-math.pi, math.pi)
        return [x_coordinate, y_coordinate, theta]

    @dbus.service.method("com.mgrobotics.OdometryInterface",
                         in_signature='', out_signature='')
    def exit(self):
        mainloop.quit()


if __name__ == '__main__':
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

    session_bus = dbus.SessionBus()
    name = dbus.service.BusName("com.mgrobotics.OdometryService", session_bus)
    state_space = StateSpace(session_bus, '/StateSpace')

    mainloop = GLib.MainLoop()
    print("Running odometry service.")
    mainloop.run()
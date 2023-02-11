import os
import sys
import math
import random

from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

file_name = "/home/filip/Desktop/informatika/mg-team-eurobot"
sys.path.append(os.path.abspath(file_name))
from simulation.import_data import *

class StateSpace(dbus.service.Object):

    @dbus.service.method("com.mgrobotics.OdometryInterface", 
                         in_signature='', out_signature='ad')
    def get_state_space(self):
        ser.write(bytearray('S', 'ascii'))
        while True:
            if ser.inWaiting():
                s = ser.readline().decode('utf').rstrip('\n')
                print(s)
                return [float(item) for item in s.split()]
    
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


class GameEngine(dbus.service.Object):

    @dbus.service.signal('com.mgrobotics.AIInterface')
    def new_desired_position(self, message):
        pass

    @dbus.service.method('com.mgrobotics.AIInterface')
    def emitt_new_desired_position(self, desired_position):
        self.new_desired_position(desired_position)
        return 'Signal emmitted'


if __name__ == '__main__':
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

    # Connect to Bus
    session_bus = dbus.SessionBus()
    name = dbus.service.BusName("com.mgrobotics.Service", session_bus)

    # Open Serial
    ser = init_serial()
    ser.open()

    state_space = StateSpace(session_bus, '/StateSpace')
    game_engine = GameEngine(session_bus, '/GameEngine')

    mainloop = GLib.MainLoop()
    print("Running odometry service.")
    mainloop.run()
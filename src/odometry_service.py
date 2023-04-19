import os
import sys
import math
import random
import argparse

from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

#Added to work anywhere
#file_name = "/home/filip/Desktop/informatika/mg-team-eurobot"
file_name = os.path.join(os.path.dirname(os.path.abspath(__file__)),os.path.pardir)
sys.path.append(os.path.abspath(file_name))
from simulation.import_data import *

parser = argparse.ArgumentParser()
parser.add_argument("-sc", "--serial_connect", default=True, action="store_false")
args = parser.parse_args()
serial_required = args.serial_connect


class StateSpace(dbus.service.Object):

    # STATE SPACE
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
    

    # AI
    @dbus.service.signal('com.mgrobotics.AI')
    def new_desired_position(self, message):
        pass

    @dbus.service.method('com.mgrobotics.AI',
                         in_signature='ad', out_signature='(a(dd)i)')
    def emit_new_desired_position(self, desired_position):
        self.new_desired_position(desired_position)
        return 'AI signal emitted'
    
    
    # PURE PURSUIT
    @dbus.service.signal('com.mgrobotics.PurePursuit')
    def new_lookahead(self, message):
        pass

    @dbus.service.method('com.mgrobotics.PurePursuit',
                         in_signature='a(ddd)', out_signature='')
    def emit_new_lookahead(self, lookahead):
        self.new_lookahead(lookahead)
        return 'Pure Pursuit signal emitted'

    @dbus.service.signal('com.mgrobotics.EmergencyStop')
    def emergency_stop(self):
        pass

    @dbus.service.method('com.mgrobotics.EmergencyStop',
                         in_signature='', out_signature='s')
    def emit_emergency_stop(self):
        self.emergency_stop()
        return 'Lidar Emergency Stop'

    @dbus.service.method("com.mgrobotics.OdometryInterface",
                         in_signature='', out_signature='')
    def exit(self):
        mainloop.quit()



if __name__ == '__main__':
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

    # Connect to Bus
    session_bus = dbus.SessionBus()
    name = dbus.service.BusName("com.mgrobotics.Service", session_bus)

    # Open Serial
    ser = None
    if serial_required:
        ser = init_serial()
        ser.open()

    # Define a service object
    state_space = StateSpace(session_bus, '/StateSpace')

    try:
        mainloop = GLib.MainLoop()
        print("Running odometry service.")
        mainloop.run()
    except KeyboardInterrupt:
        state_space.exit()
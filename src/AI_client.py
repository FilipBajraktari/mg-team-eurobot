import time
import odrive
import numpy as np
from queue import Queue
from threading import Thread

import dbus
import dbus.service
import dbus.mainloop.glib

from Behaviors.Behavior_template import *

iface = None
iface_ai = None
CancelRequired = object()

def Master(x):
    print("working")
    while not x.Complete:
        x.ControlLoop(CancelRequired)
    if x.Error:
        print(x.Error)
    return


def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()
 
    remote_object = bus.get_object("com.mgrobotics.Service",
                                   "/StateSpace")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    iface_ai = dbus.Interface(remote_object, "com.mgrobotics.AI")

    odrv0 = odrive.find_any()

    worker = None
    while True:
        # AI STUFF
        # current_state_space = iface.get_random_state_space()
        # iface_ai.emit_new_desired_position([0,0,0])

        if not worker:
            behaviour = TurnAbsolute(iface, iface_ai, odrv0, np.pi/2)
            worker = Thread(target=Master, args=(behaviour,))
            worker.daemon = True
            worker.start()
        if worker.is_alive() == False:
            worker = None
            print("Im finished ;)")
        
        time.sleep(.1)
        

if __name__ == '__main__':
    main()
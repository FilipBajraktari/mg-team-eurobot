import time
import odrive
import numpy as np
from queue import Queue
from threading import Thread

import dbus
import dbus.service
import dbus.mainloop.glib

from Behaviors.Behavior import *

iface = None
iface_ai = None
CancelRequired = False

def Master(action_queue):
    print("Master started working.")
    while True:
        behaviour = action_queue.get()
        while not behaviour.Complete:
            behaviour.ControlLoop(CancelRequired)
        if behaviour.Error:
            print(behaviour.Error)

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()
 
    remote_object = bus.get_object("com.mgrobotics.Service",
                                   "/StateSpace")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    iface_ai = dbus.Interface(remote_object, "com.mgrobotics.AI")

    odrv0 = odrive.find_any()

    action_queue = Queue()
    worker = Thread(target=Master, args=(action_queue, ))
    worker.daemon = True
    worker.start()
    fill_the_stack = True
    while True:
        # AI STUFF
        # current_state_space = iface.get_random_state_space()
        # iface_ai.emit_new_desired_position([0,0,0])

        if fill_the_stack:
            # behaviour = TurnRelative(iface, iface_ai, odrv0, 3*np.pi/2)
            behaviour = MoveRelative(iface, iface_ai, odrv0, -10)
            action_queue.put(behaviour)
            fill_the_stack = False
        
        time.sleep(.1)
        

if __name__ == '__main__':
    main()
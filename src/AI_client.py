import time

import threading
import dbus
import dbus.service
import dbus.mainloop.glib
from Behaviors.Behavior_template import *

iface = None
iface_ai = None
CancelRequired = False

def Action(x):
    print("working")
    global CancelRequired
    StateSpace = None
    while not (x.Complete):
        x.ControlLoop(StateSpace,CancelRequired)
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



    x = Template_Controller(iface = iface)
    worker = None
    while True:
        # AI STUFF
        current_state_space = iface.get_random_state_space()
        # iface_ai.emit_new_desired_position([0,0,0])
        if not worker:
            worker = threading.Thread(target = Action, args=(Template_Controller(iface = iface),))
            worker.daemon=True
            worker.start()
        if worker.is_alive() == False:
            worker = None
            print("Im finished ;)")
        
        print("Thinking...")
        time.sleep(1)
        

if __name__ == '__main__':
    main()
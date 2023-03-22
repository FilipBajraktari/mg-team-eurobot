import time

import dbus
import dbus.service
import dbus.mainloop.glib

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()
 
    remote_object = bus.get_object("com.mgrobotics.Service",
                                   "/StateSpace")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    iface_ai = dbus.Interface(remote_object, "com.mgrobotics.AI")

    while True:
        # AI STUFF
        current_state_space = iface.get_random_state_space()
        iface_ai.emit_new_desired_position([0,0,0])
        time.sleep(1)

if __name__ == '__main__':
    main()
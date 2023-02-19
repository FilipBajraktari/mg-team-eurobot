import time
import threading

from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

# LOOKAHEAD POSITION COORDINATES
x_coordinate = None
y_coordinate = None
theta = None

def catchall_new_lookahead(lookahead_postion):
    global x_coordinate, y_coordinate, theta
    x_coordinate, y_coordinate, theta = lookahead_postion

def pure_pursuit(iface):
    time.sleep(0.5) # Time to setup Glib mainloop

    while True:
        # print("Lookahead position:\t", x_coordinate, y_coordinate, theta, sep=" ")
        # print(type(iface.get_random_state_space()[0]))
        time.sleep(1)

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()

    remote_object = bus.get_object("com.mgrobotics.Service",
                                   "/StateSpace")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    bus.add_signal_receiver(catchall_new_lookahead, dbus_interface = "com.mgrobotics.PurePursuit")

    # Define a thread that will concurrently run with mainloop
    # This thread will handle Pure Pursuit algorithm
    t = threading.Thread(target=pure_pursuit, args=(iface,))
    t.daemon = True
    t.start()

    # Define a mainloop that will handle dbus events
    mainloop = GLib.MainLoop()
    print("Running pure pursuit.")
    mainloop.run()

if __name__ == '__main__':
    main()
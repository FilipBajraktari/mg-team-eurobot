import time
import threading

from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

# LOOKAHEAD POSITION COORDINATES
x_lookahead = None
y_lookahead = None
theta_lookahead = None

# EMERGENCY STOP
estop = False

def catchall_new_lookahead(lookahead_postion):
    global x_lookahead, y_lookahead, theta_lookahead
    x_lookahead, y_lookahead, theta_lookahead = lookahead_postion

def catchall_estop():
    global estop
    estop = not estop

def pure_pursuit(iface, lidar_iface):
    time.sleep(0.5) # Time to setup Glib mainloop
    
    while True:
        # print("Lookahead position:\t", x_lookahead, y_lookahead, theta_lookahead, sep=" ")
        # print(type(iface.get_random_state_space()[0]))

        print(estop)
        #print(lidar_iface.opponents_coordinates(3))
        time.sleep(.1)

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    session_bus = dbus.SessionBus()

    remote_object = session_bus.get_object("com.mgrobotics.Service", "/StateSpace")
    lidar_object = session_bus.get_object("com.mgrobotics.LidarService", "/Lidar")

    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    lidar_iface = dbus.Interface(lidar_object, "com.mgrobotics.LidarInterface")

    # bus.add_signal_receiver(catchall_new_lookahead, dbus_interface = "com.mgrobotics.PurePursuit")
    session_bus.add_signal_receiver(catchall_estop, dbus_interface = "com.mgrobotics.EmergencyStop")

    # Define a thread that will concurrently run with mainloop
    # This thread will handle Pure Pursuit algorithm
    t = threading.Thread(target=pure_pursuit, args=(iface, lidar_iface))
    t.daemon = True
    t.start()

    # Define a mainloop that will handle dbus events
    mainloop = GLib.MainLoop()
    print("Running pure pursuit.")
    mainloop.run()

if __name__ == '__main__':
    main()

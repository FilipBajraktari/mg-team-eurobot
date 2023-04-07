import time
import threading

from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

import matplotlib.pyplot as plt
from canvas_testing import custom_canvas
import _rtrrt as rt
import random

# DESIRED POSITION COORDINATES
x_desired = None
y_desired = None
theta_desired = None
iface_pp = None
iface = None
xOffset = 1500
yOffset = 1000

def catchall_new_desired_position(desired_position):
    global x_desired, y_desired
    x_desired, y_desired = desired_position
def rrtSend():
    Obstacles = []
    while iface == None: 
        time.sleep(0.1)
    ncords=iface.get_state_space()
    ncords[0]*=10
    ncords[0]+= xOffset
    ncords[1]*=10
    ncords[1]+= yOffset
    return (ncords[0],ncords[1], x_desired,y_desired,Obstacles,len(Obstacles))

def rrtRecv(path,Tree):
    global iface_pp
    if iface_pp == None:
        return
    iface_pp.emit_new_lookahead([(x[0]/10*4,x[1]/10*4) for x in path])
    #Map = Tree
    #friendBOT.lastWaypoint=0

def rrt_star(iface, iface_pp):
    time.sleep(2)
    print("started rrt")
    rt.startRRT(rrtSend, rrtRecv)

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()
 
    remote_object = bus.get_object("com.mgrobotics.Service",
                                   "/StateSpace")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    iface_pp = dbus.Interface(remote_object, "com.mgrobotics.PurePursuit")
    bus.add_signal_receiver(catchall_new_desired_position, dbus_interface = "com.mgrobotics.AI")

    # Define a thread that will concurrently run with mainloop
    # This thread will handle RRT star algorithm
    t = threading.Thread(target=rrt_star, args=(iface, iface_pp))
    t.daemon = True
    t.start()

    # Define a mainloop that will handle dbus events
    mainloop = GLib.MainLoop()
    print("Running RRT* algorithm.")
    mainloop.run()

if __name__ == '__main__':
    main()
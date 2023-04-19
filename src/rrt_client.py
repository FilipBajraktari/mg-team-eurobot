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
waypoints = None
x_desired = None
y_desired = None
theta_desired = None
iface = None
lidar_iface = None
xOffset = 1500
yOffset = 1000
lock = False


def rrtSend():
    global lidar_iface, iface, lock
    Obstacles = lidar_iface.opponents_coordinates(2)
    Obstacles = [(O.x*10+xOffset-250,O.y*10+yOffset-250,200) for O in Obstacles]
    while iface == None: 
        time.sleep(0.1)
    ncords=iface.get_state_space()
    ncords[0]*=10
    ncords[0]+= xOffset-250
    ncords[1]*=10
    ncords[1]+= yOffset-250
    if  round(x_desired,3) == -125.1 and round(y_desired,3) == -75.1:
        lock=False
        x_d = -1
        y_d = -1
    else:
        x_d = 10 * x_desired
        x_d+= xOffset-250
        y_d = 10 * y_desired
        y_d+= yOffset-250
    return (ncords[0], ncords[1], x_d,y_d,Obstacles,len(Obstacles))

def rrtRecv(path,Tree):
    global waypoints
    waypoints = path
    #Map = Tree
    #friendBOT.lastWaypoint=0

def rrt_star():
    time.sleep(2)
    print("started rrt")
    rt.startRRT(rrtSend, rrtRecv)


class RRT(dbus.service.Object):

    @dbus.service.method("com.mgrobotics.RrtInterface",
                         in_signature='(dd)', out_signature='s')
    def SetDesiredPosition(self, Goal):
        global x_desired,y_desired, lock
        if  round(Goal[0],3) == -125.1 and round(Goal[1],3) == -75.1:
            lock=True
        x_desired,y_desired = Goal
        return "Goal set"
    
    @dbus.service.method("com.mgrobotics.RrtInterface",
                         in_signature='', out_signature='a(dd)')
    def GetWaypoints(self):
        return [((x[0]+250-1500)/10,(x[1]+250-1000)/10) for x in waypoints]

    @dbus.service.method("com.mgrobotics.RrtInterface",
                         in_signature='', out_signature='')
    def exit(self):
        mainloop.quit()

if __name__ == '__main__':
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()
    name = dbus.service.BusName("com.mgrobotics.RrtService", bus)
    rrt_dbus = RRT(bus, '/rtRRT')
 
    remote_object = bus.get_object("com.mgrobotics.Service", "/StateSpace")
    lidar_object = bus.get_object("com.mgrobotics.LidarService", "/Lidar")

    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    lidar_iface = dbus.Interface(lidar_object, "com.mgrobotics.LidarInterface")


    # Define a thread that will concurrently run with mainloop
    # This thread will handle RRT star algorithm
    t = threading.Thread(target=rrt_star, args=())
    t.daemon = True
    t.start()

    # Define a mainloop that will handle dbus events
    mainloop = GLib.MainLoop()
    print("Running RRT* algorithm.")
    mainloop.run()
import numpy as np
import time
import threading
from sklearn.cluster import KMeans

from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

from fastestrplidar import FastestRplidar
import fastestrplidar
import multiprocessing as mp
from multiprocessing import Queue
import queue

# ODrive variables
IDLE = 1
CLOSED_LOOP_CONTROL = 8


# Create a Lock


def in_table(x, y):
    WIDTH = 3000
    HEIGHT = 2000
    ERROR = 50

    if -WIDTH//2 + ERROR < x and x < WIDTH//2 - ERROR:
        if -HEIGHT//2 + ERROR < y and y < HEIGHT//2 - ERROR:
            return True
    return False

def in_collision_circle(x_robot, y_robot, x, y, COLLISION_RANGE = 200):
    return np.square(x-x_robot) + np.square(y-y_robot) <= COLLISION_RANGE**2

def in_collision_FOV(angle, distance, OPENING_ANGLE = np.pi/4, COLLISION_RANGE = 200):
    return (angle<=OPENING_ANGLE or 2*np.pi-angle<=OPENING_ANGLE) and distance<=COLLISION_RANGE

points = []
def lidar_main(Q:mp.Queue,q1:Queue,q2:Queue):
    time.sleep(0.5) # Time to setup Glib mainloop
    lidar = FastestRplidar()
    lidar.connectlidar()
    lidar.stopmotor()
    lidar = FastestRplidar()
    lidar.connectlidar()
    lidar.startmotor(my_scanmode=2)
    
    #result = lidar.fetchscandata()

    #lidar.stopmotor
    #print ("result - ")
    #print (result)
    #exit(1)
    print("Lidar successfully connected.")
    # Connect to ODrive
    # my_drive = odrive.find_any()
    my_drive = None
    print("ODrive successfully connected.")
    
    robot_stop_moving = False
    while True:
    
        should_move = True
        x_robot, y_robot, theta_robot, _, _ = q1.get()
        while True:
            try:
                _robot, y_robot, theta_robot, _, _  = q1.get_nowait()
            except queue.Empty:
                break
        
        print(x_robot, y_robot, theta_robot)
        x_robot -= 24
        #x_robot, y_robot, theta_robot = 0, 0, 0
        result = lidar.get_scan_as_vectors(filter_quality=True)

        
        global points
        points = []
        for angle, distance, quality in result:
            angle = np.radians(angle) - 3*np.pi/4
            if angle < 0: 
                angle += 2*np.pi
            x_lidar, y_lidar = distance*np.cos(angle), -distance*np.sin(angle)
            x_prime = x_lidar*np.cos(theta_robot) - y_lidar*np.sin(theta_robot)
            y_prime = x_lidar*np.sin(theta_robot) + y_lidar*np.cos(theta_robot)
            x, y = x_robot + x_prime, y_robot + y_prime
            if in_table(x, y):
                # if in_collision_circle(x_robot, y_robot, x, y):
                if in_collision_FOV(angle, distance):
                    if not robot_stop_moving:
                        q2.put(None)
                    should_move = False
                    robot_stop_moving = True
                    #print("Too close!")
                #print(x,y)
                points.append((x,y))
        try:
            Q.put(points)
        except queue.Full:
            print("ffull")

        if robot_stop_moving and should_move:
            q2.put(None)
            robot_stop_moving = False
            print("I got away!!!")
        time.sleep(.1)

def comparePol(x:tuple,y:tuple):
    return False      

class Lidar(dbus.service.Object):

    Q : Queue = Queue()

    @dbus.service.method("com.mgrobotics.LidarInterface",
                         in_signature='i', out_signature='a(dd)')
    def opponents_coordinates(self, number_of_clusters=1):
        
        points = []
        while True:
            try:
                points = self.Q.get_nowait()
            except queue.Empty:
                break
        points_cpy = []

        

        
        if len(points) < number_of_clusters:
            return points
        else:
            points_cpy = points

        

        if points_cpy == []:
            return points_cpy
        
        if number_of_clusters == 1:
            x = 0
            y = 0
            for point in points_cpy:
                x += point[0]
                y += point[1]
            n = len(points_cpy)
            return [(x/n, y/n)]
        
        # Clusterization
        kmeans = KMeans(n_clusters=number_of_clusters)
        kmeans.fit(points_cpy)
        labels = kmeans.labels_

        clusters = [[0, 0, 0] for i in range(number_of_clusters)]
        for x,point in zip(enumerate(labels),points_cpy):
            i, label = x 
            clusters[label][0] += 1
            clusters[label][1] += point[0]
            clusters[label][2] += point[1]

        opponents = [(cluster[1]/cluster[0], cluster[2]/cluster[0]) 
                    for cluster in clusters]
        
        return opponents
    
    @dbus.service.method("com.mgrobotics.LidarInterface",
                         in_signature='', out_signature='')
    def exit(self):
        mainloop.quit()

def SendToProcess(Q2,q,odiface,estop_iface):

    while True:
        Q2.put(odiface.get_state_space())
        while True:
                try:
                    points = q.get_nowait()
                    estop_iface.emit_emergency_stop()
                except queue.Empty:
                    break
        time.sleep(0.1)


if __name__ == "__main__":
    # Setup DBus
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    session_bus = dbus.SessionBus()
    name = dbus.service.BusName("com.mgrobotics.LidarService", session_bus)
    lidar_dbus = Lidar(session_bus, '/Lidar')

    remote_object = session_bus.get_object("com.mgrobotics.Service",
                                           "/StateSpace")
    odometry_iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    estop_iface = dbus.Interface(remote_object, "com.mgrobotics.EmergencyStop")

    
    #lidar = FastestRplidar()
    #lidar.connectlidar()
    #lidar.stopmotor()
    #lidar = FastestRplidar()
    #lidar.connectlidar()
    #lidar.stopmotor()
    #lidar = FastestRplidar()
    #print("Fast")
    #lidar.connectlidar()
    #lidar.stopmotor()
    ## Lidar initialization 
    #print("pre")
    


    # Define a thread that will concurrently run with mainloop
    # This thread will handle Lidar
    q1 = Queue()
    q2 = Queue()
    
    t = mp.Process(target=lidar_main, args=(lidar_dbus.Q,q1,q2))
    t.daemon = True
    t.start()

    t1 = threading.Thread(target=SendToProcess, args=(q1,q2,odometry_iface,estop_iface))
    t1.daemon = True
    t1.start()
    # Define a mainloop that will handle dbus events
    try:
        mainloop = GLib.MainLoop()
        print("Running lidar.")
        mainloop.run()
    except KeyboardInterrupt:
        lidar_dbus.exit()

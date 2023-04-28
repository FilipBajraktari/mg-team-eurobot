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
from functools import cmp_to_key
import glm

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

def in_collision_FOV(angle, distance, OPENING_ANGLE = np.pi/4, COLLISION_RANGE = 300):
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
    try:
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
        
            #print(x_robot, y_robot, theta_robot)
            
            
            x_robot *= 10
            y_robot *= 10
            a = glm.rotate((60,0),theta_robot)
            x_robot+=a.x
            y_robot+=a.y
            #x_robot, y_robot, theta_robot = 0, 0, 0
            result = lidar.get_scan_as_vectors(filter_quality=True)

            
            global points
            points = []
            for angle, distance, quality in result:
                angle = np.radians(angle) - 3*np.pi/4
                #distance+= 40
            
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
                            q2.put(False)
                        should_move = False
                        robot_stop_moving = True
                        print("Too close!")
                    #print(x,y)
                    points.append((x,y))
            try:
                Q.put((glm.vec2(x_robot,y_robot),points))
            except queue.Full:
                print("ffull")

            if robot_stop_moving and should_move:
                q2.put(False)
                robot_stop_moving = False
                print("I got away!!!")
            time.sleep(.1)
    except KeyboardInterrupt:
        lidar.stopmotor()
def Quad(p):
    quad = ((0,3),(1,2))
    return quad[1 if p[0]>=0 else 0][1 if p[1]>=0 else 0]

def comparePol(x:tuple,y:tuple):
    
    return x[0] * y[1] > x[1] * y[0] if Quad(x) == Quad(y) else Quad(x)<Quad(y)

rob : glm.vec2
points : list[glm.vec2] = []

class Lidar(dbus.service.Object):

    Q : Queue = Queue()

    @dbus.service.method("com.mgrobotics.LidarInterface",
                         in_signature='i', out_signature='a(dd)')
    def opponents_coordinates(self, number_of_clusters=1):
        global rob,points
        
        while True:
            try:
                rob,points = self.Q.get_nowait()
            except queue.Empty:
                break
        points_cpy = []

        

        
        
        points_cpy = points
        
        if number_of_clusters == 1 and False:
            x = 0
            y = 0
            for point in points_cpy:
                x += point[0]
                y += point[1]
            n = len(points_cpy)
            return [(x/n, y/n)]
        if(len(points)<1):
            print("help")
            return []
        sorted(points, key=cmp_to_key(lambda x,y:comparePol(glm.vec2(x) - rob,glm.vec2(y) - rob)))
        prev=points_cpy[0]
        t = glm.vec2(0)
        n = 0
        op = []
        for point in points:
            if glm.distance2(prev,point) < 100**2:
                n+=1
                t+=glm.vec2(point)
            else:
                op.append((t/n).to_tuple())
                n = 1
                t = glm.vec2(point)
            prev = point
        if n>0:
            op.append((t/n).to_tuple())
        #print(op)
        return op
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

                    print("stop/start")
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

import numpy as np
import time
import threading
import odrive
from sklearn.cluster import KMeans

from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

from fastestrplidar import FastestRplidar

# ODrive variables
IDLE = 1
CLOSED_LOOP_CONTROL = 8

# Create a Lock
lock = threading.Lock()

def in_table(x, y):
    WIDTH = 600
    HEIGHT = 400
    ERROR = 5

    if -WIDTH//2 + ERROR < x and x < WIDTH//2 - ERROR:
        if -HEIGHT//2 + ERROR < y and y < HEIGHT//2 - ERROR:
            return True
    return False

def in_collision_circle(x_robot, y_robot, x, y, COLLISION_RANGE = 0):
    return np.sqrt(np.square(x-x_robot) + np.square(y-y_robot)) <= COLLISION_RANGE

def in_collision_FOV(angle, distance, OPENING_ANGLE = np.pi/4, COLLISION_RANGE = 0):
    return (angle<=OPENING_ANGLE and 2*np.pi-OPENING_ANGLE<=OPENING_ANGLE) and distance<=COLLISION_RANGE

points = []
def lidar_main(iface, lidar):
    time.sleep(0.5) # Time to setup Glib mainloop

    # Connect to ODrive
    # my_drive = odrive.find_any()
    my_drive = None
    print("ODrive successfully connected.")
    
    robot_stop_moving = False
    while True:
        should_move = True
        x_robot, y_robot, theta_robot = iface.get_random_state_space()
        x_robot, y_robot, theta_robot = 0, 0, -np.pi/2
        result = lidar.get_scan_as_vectors(filter_quality=True)

        with lock:
            global points
            points = []
            for angle, distance, quality in result:
                angle = np.radians(angle)
                x_lidar, y_lidar = distance*np.cos(angle), -distance*np.sin(angle)
                x_prime = x_lidar*np.cos(theta_robot) - y_lidar*np.sin(theta_robot)
                y_prime = x_lidar*np.sin(theta_robot) + y_lidar*np.cos(theta_robot)
                x, y = x_robot + x_prime, y_robot + y_prime
                if in_table(x, y):
                    if in_collision_circle(x_robot, y_robot, x, y):
                        my_drive.axis0.controller.input_vel = 0
                        my_drive.axis1.controller.input_vel = 0
                        my_drive.axis0.requested_state = IDLE
                        my_drive.axis1.requested_state = IDLE
                        should_move = False
                        robot_stop_moving = True
                        print("Too close!")
                    points.append((x,y))

            if robot_stop_moving and should_move:
                my_drive.axis0.requested_state = CLOSED_LOOP_CONTROL
                my_drive.axis1.requested_state = CLOSED_LOOP_CONTROL
                robot_stop_moving = False
        time.sleep(.1)

class Lidar(dbus.service.Object):

    @dbus.service.method("com.mgrobotics.LidarInterface",
                         in_signature='i', out_signature='a(dd)')
    def opponents_coordinates(self, number_of_clusters):
        points_cpy = []
        with lock:
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
        
        # return points_cpy
        # Clusterization
        kmeans = KMeans(n_clusters=number_of_clusters)
        kmeans.fit(points_cpy)
        labels = kmeans.labels_

        # (number_of_points_in_cluster, sum_of_x, sum_of_y)
        clusters = [[0, 0, 0] for i in range(number_of_clusters)]
        for i, label in enumerate(labels):
            clusters[label][0] += 1
            clusters[label][1] += points_cpy[i][0]
            clusters[label][2] += points_cpy[i][1]

        opponents = [(cluster[1]/cluster[0], cluster[2]/cluster[0]) 
                    for cluster in clusters]
        return opponents
    
    @dbus.service.method("com.mgrobotics.LidarInterface",
                         in_signature='', out_signature='')
    def exit(self):
        mainloop.quit()


if __name__ == "__main__":
    # Setup DBus
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    session_bus = dbus.SessionBus()
    name = dbus.service.BusName("com.mgrobotics.LidarService", session_bus)
    lidar_dbus = Lidar(session_bus, '/Lidar')

    remote_object = session_bus.get_object("com.mgrobotics.Service",
                                           "/StateSpace")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")


    # Lidar initialization
    lidar = FastestRplidar()
    lidar.connectlidar()
    lidar.startmotor(my_scanmode=2)
    print("Lidar successfully connected.")


    # Define a thread that will concurrently run with mainloop
    # This thread will handle Lidar
    t = threading.Thread(target=lidar_main, args=(iface, lidar))
    t.daemon = True
    t.start()

    # Define a mainloop that will handle dbus events
    try:
        mainloop = GLib.MainLoop()
        print("Running lidar.")
        mainloop.run()
    except KeyboardInterrupt:
        lidar.stopmotor()
        lidar_dbus.exit()
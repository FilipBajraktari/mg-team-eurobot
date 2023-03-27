from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

from sklearn.cluster import KMeans

def main():
    bus = dbus.SessionBus()

    remote_object = bus.get_object("com.mgrobotics.LidarService",
                                   "/Lidar")
    iface = dbus.Interface(remote_object, "com.mgrobotics.LidarInterface")

    number_of_clusters = 2
    points_cpy = iface.opponents_coordinates(number_of_clusters)
    print(points_cpy)

if __name__ == '__main__':
    main()

# from sklearn.cluster import KMeans

# points_cpy = [(0,0),(0.1,0.1),(1,1),(1.1,1.2)]*10000
# kmeans = KMeans(n_clusters=2)
# kmeans.fit(points_cpy)
# labels = kmeans.labels_
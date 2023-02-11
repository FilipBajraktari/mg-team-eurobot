import dbus

def main():
    bus = dbus.SessionBus()

    remote_object = bus.get_object("com.mgrobotics.Service",
                                   "/GameEngine")
    iface = dbus.Interface(remote_object, "com.mgrobotics.AIInterface")
    
    iface.emitt_new_desired_position([0, 0, 0])

if __name__ == '__main__':
    main()
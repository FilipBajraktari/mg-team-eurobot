import time
import odrive
import numpy as np
from queue import Queue
from threading import Thread

import dbus
import dbus.service
import dbus.mainloop.glib

from gi.repository import GLib

from Behaviors.Behavior import *

# EMERGENCY STOP
estop = False
done = 0
def catchall_estop():
    global estop,done
    estop = not estop

CancelRequired = False

def Master(action_queue):
    print("Master started working.")
    while True:
        behaviour = action_queue.get()
        while not behaviour.Complete:
            #print(estop)
            behaviour.ControlLoop((CancelRequired))
        if behaviour.Error:
            print(behaviour.Error)
        print("Action is FINISHED ;)")
        done += 1
def GetSerialConnection():
    import serial
    import serial.tools.list_ports


    # Init Pyserial
    ports = serial.tools.list_ports.comports()
    ser = serial.Serial()
    portList = []



    for p in ports:
        portList.append(str(p))
        print(str(p))


    #val = input("select port: /dev/ttyUSB")
    val = 0
    portVar = None

    for x in portList:
        if x.find("ttyUSB") < 0:
            continue
        if x.find("Serial")<0:
            continue 
        portVar = x.split()[0]

    
    ser.baudrate = 115200
    ser.port = portVar
    ser.open()
    while True:
        command = input("Type in a command: ")
        if(command == "close"):
            break
        ser.write(bytearray(command, 'ascii'))
        time.sleep(0.1)
        if ser.inWaiting():
            s = ""
            while ser.inWaiting():
                s += ser.read().decode('ascii')
            print(s)
    return ser



def main(): 
    global done
    ser = GetSerialConnection()
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()
 
    remote_object = bus.get_object("com.mgrobotics.Service", "/StateSpace")
    lidar_object = bus.get_object("com.mgrobotics.LidarService", "/Lidar")
    rrt_object = bus.get_object("com.mgrobotics.RrtService", "/rtRRT")

    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    iface_ai = dbus.Interface(remote_object, "com.mgrobotics.AI")
    ifaceRrt = dbus.Interface(rrt_object, "com.mgrobotics.RrtInterface")
    ifaceLidar = dbus.Interface(lidar_object, "com.mgrobotics.LidarInterface")

    bus.add_signal_receiver(catchall_estop, dbus_interface = "com.mgrobotics.EmergencyStop")

    odrv0 = odrive.find_any()

    action_queue = Queue()
    worker = Thread(target=Master, args=(action_queue, ))
    worker.daemon = True
    worker.start()
    fill_the_stack = True
    
    while True:
        # AI STUFF
        #current_state_space = iface.get_random_state_space()
        #iface_ai.emit_new_desired_position([0,0,0])

        if fill_the_stack:
            # behaviour = TurnRelative(iface, iface_ai, odrv0, 3*np.pi/2)
            #behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, ser, "pu 1", 1)
            #action_queue.put(behaviour)
            behaviour = Start(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0)
            action_queue.put(behaviour)
            behaviour = MoveRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, 41)
            
            action_queue.put(behaviour)
            behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,"pu 2",4)
            action_queue.put(behaviour)
            
            behaviour = MoveRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, 18)
            action_queue.put(behaviour)

            behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,"pu 1",4)
            action_queue.put(behaviour)
            
            behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,"dr 1",4)
            action_queue.put(behaviour)

            behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,"dr 2",4)
            action_queue.put(behaviour)
            Args = vec2(-150+112,-100+72)
            behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,Args,TargetCake)
            action_queue.put(behaviour)
            fill_the_stack = False
        if done == 1:
            cake = vec2(-150+112,-100+72)
            #a,b,c,_,_
        
        time.sleep(.1)
        

if __name__ == '__main__':
    main()

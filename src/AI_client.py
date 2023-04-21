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

HOMOLOGATION = False

estop = False
done = 0
def catchall_estop():
    global estop

    estop = not estop
    print(estop)

CancelRequired = False
Timeout = False
Time = 0

def Master(action_queue:Queue):
    global done
    print("Master started working.")
    while True:
            
        behaviour = action_queue.get()
        while not behaviour.Complete:
            #print(estop)
            behaviour.ControlLoop((CancelRequired or estop or Timeout))
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

k = True

def main(): 
    global done, iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,Time, Timeout
    
    while k:
        time.sleep(0.1)
    time.sleep(1)

    action_queue = Queue()
    worker = Thread(target=Master, args=(action_queue, ))
    worker.daemon = True
    worker.start()
    fill_the_stack = True
    
    while True:
        # AI STUFF
        #current_state_space = iface.get_random_state_space()
        #iface_ai.emit_new_desired_position([0,0,0])
        if done>0 and Time==0:
            Time=time.time()
        if Time >0 and time.time()-Time>95:
            Timeout = True
            print("Finished")
        #fill_the_stack = False
        #if(action_queue.empty()):
            #behaviour = TurnRelative(iface, iface_ai, odrv0, 1*np.pi/4)
            #action_queue.put_nowait(behaviour)
        if fill_the_stack:
            # behaviour = TurnRelative(iface, iface_ai, odrv0, 3*np.pi/2)
            #behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, ser, "pu 1", 1)
            #action_queue.put(behaviour)
            behaviour = Start(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0)
            action_queue.put(behaviour)
            if HOMOLOGATION:
                #behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,"ch 2",1)
                #action_queue.put(behaviour)
                Args = vec2(150-26,100-26)
                #behaviour = Traverse ()
                behaviour = MoveRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, 130)
                action_queue.put(behaviour)

                Args = vec2(-0+15,-60)
                behaviour = Traverse(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,Args,TargetExact)
                #action_queue.put(behaviour)

                behaviour = MoveRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, -130)
                action_queue.put(behaviour)

                Args = vec2(35-150,35-100)
                behaviour = Traverse(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,Args,TargetExact)
                #action_queue.put(behaviour)

                Args = vec2(-0+15,-60)
                behaviour = Traverse(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,Args,TargetExact)
                #action_queue.put(behaviour)
            else:
                behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,"ch 2",1)
                action_queue.put(behaviour)
                behaviour = MoveRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, 39)
                
                action_queue.put(behaviour)
                behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,"pu 0",5)
                action_queue.put(behaviour)
                
                
                behaviour = MoveRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, 20)
                action_queue.put(behaviour)

                behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,"pu 2",4)
                action_queue.put(behaviour)
                
                behaviour = TurnRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, glm.pi())
                action_queue.put(behaviour)

                behaviour = MoveRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, 40)
                action_queue.put(behaviour)

                behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,"dr 2",4)
                action_queue.put(behaviour)

                

                behaviour = MoveRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, -20)
                action_queue.put(behaviour)


                behaviour = CommandCakeThing(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ser,"dr 0",5)
                action_queue.put(behaviour)

                

                behaviour = TurnRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0, glm.pi())
                action_queue.put(behaviour)

                
                Args = vec2(-150+112.5,-100+72.5)
                Args = vec2(-0+15,-60)
                Args.y = Args.y
                behaviour = Traverse(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,Args,TargetExact)
                action_queue.put(behaviour)
            fill_the_stack = False
        if done == -1:
            if not HOMOLOGATION:
                cake = vec2(-150+112.5,(-100+72.5))
                xro,yro,thetaro,_,_ = iface.get_state_space()
                ro = vec2(xro,yro)
                robo = RoboT(ro,None,27)
                cakeoff = robo.toLocalSystem(cake)
                ta = glm.atan(cakeoff.y,cakeoff.x)
                behaviour = TurnRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,ta)
                action_queue.put(behaviour)
                movd = glm.length(cakeoff)
                behaviour = TurnRelative(iface, iface_ai,ifaceRrt,ifaceLidar, odrv0,movd)
                action_queue.put(behaviour)
            #a,b,c,_,_
        
        
        

if __name__ == '__main__':
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
    t1 = Thread(target=main, args=())
    t1.daemon = True
    t1.start()
    k=False
    try:
        mainloop = GLib.MainLoop()
        print("Running lidar.")
        mainloop.run()
    except KeyboardInterrupt:
        bus.exit()

    time.sleep(.1)
"""
A program to visualize pathfinding simulations for a differential 
drive robot in python implented in pygame. Made for Eurobot2023

GOALS:  1. click robots to make them try to path find
        2. Show planned path for robots
                
        3. Test for collisions between them
        
Implementation : 1. * Add map, make some sort of graphics for each of the elements
                 2. For starters allow users to select waypoints for enemy bots, and goal for our bot...
                 3. Generate waypoints for our robot
                 4. Figure out a path planning algorithm
                 5. Add some kind of check for Collision
"""

## Variables ##
WINDOW_SIZE = (1200,800)
LOCALTESTING = True
NO_ODOMETRY = True

## Imports ##
import tarfile
from tracemalloc import start
from xml.dom.expatbuilder import theDOMImplementation
import numpy, sys, pygame, math, cmath
from pygame import *
from typing import List
from Robot import RoboT
from Obstacle import Obstacle
import time
import threading
import sys,os

if not LOCALTESTING:
    import odrive
    my_drive = odrive.find_any()
if not NO_ODOMETRY:
    from gi.repository import GLib

    import dbus
    import dbus.service
    import dbus.mainloop.glib

## INIT ##
file_name = os.path.dirname(os.path.abspath(__file__))



pygame.init()
__clock__ = pygame.time.Clock()
__fb__ = pygame.display.set_mode(WINDOW_SIZE,0,32)
__bg__ = pygame.image.load(os.path.join(file_name,'table.png'))
__bg__ = pygame.transform.rotate(__bg__, 90).copy()
__bg__ = pygame.transform.scale(__bg__, (1200,800))
BotPic1 = pygame.transform.scale(pygame.image.load(os.path.join(file_name,r'OurRobot.png')), (2*4*19.1,2*4*10))
BotPic1 = pygame.transform.rotate(BotPic1, 180)
BotPic2 = pygame.transform.scale(pygame.image.load(os.path.join(file_name,r'EvilRobot.png')), (2*4*19.1,2*4*19.1))
BotPic2 = pygame.transform.rotate(BotPic2, 180)

PAUSED = 1
RUNNING = 2
SELECT = 3
OBSTACLE = 4
WAYPOINTS = 5
ROBOT = 6


state = PAUSED
#data = numpy.empty(WINDOW_SIZE[0]*WINDOW_SIZE[1]*4, dtype=numpy.int8)
#cairo_surface = cairo.ImageSurface.create_for_data(data, cairo.FORMAT_ARGB32, WINDOW_SIZE[0], WINDOW_SIZE[1],*4)

#ctx = cairo.Context(cairo_surface)

## Helper Classes ##

# TODO: Change wheel velocity to wheel rotational velocity



## ROBOT PATH tracking
def sgn (num):
    if num >= 0:
            return 1
    else: 
        return -1

def turnAngle(target,current):
    current = numpy.degrees(current)
    er = target - current
    
    if -181 > er or er > 181:
        er = er + sgn(er)*(-360) 
    return numpy.radians(er)

def PurePursuit(robot: RoboT,dt) -> tuple[float, float]:
    karot = None
    if(len(robot.waypoints)==0):
        return 0,0
    for i in range(robot.lastWaypoint+1, len(robot.waypoints)):
        p = Intersect((robot.x,robot.y), robot.waypoints[robot.lastWaypoint], robot.waypoints[i],20)
        if(p==-1):
            karot = robot.waypoints[robot.lastWaypoint]
            break
        elif p!=-2:
            karot = p
            break
        ##if(math.dist(robot.waypoints[i],(robot.x,robot.y))>60):
          ##  karot = robot.waypoints[i]
           ## break
        robot.lastWaypoint+=1
    if(len(robot.waypoints)<=robot.lastWaypoint+1) and (robot.x-robot.waypoints[-1][0])**2+(robot.y-robot.waypoints[-1][1])**2<=402:
        karot=robot.waypoints[-1]
        
    if(len(robot.waypoints)<=robot.lastWaypoint+1) and (robot.x-robot.waypoints[-1][0])**2+(robot.y-robot.waypoints[-1][1])**2>5:
        karot=None
        
    
    if(karot==None): 
        
        return (0,0)
    
    velocityKp = 5
    omegaKp = 10
    omegaKd = 1
    omegaTau = 0.5
    
    linearError = numpy.sqrt((karot[0]-robot.x)**2+(karot[1]-robot.y)**2)
    targetAngle = numpy.arctan2(karot[0]-robot.x, karot[1]-robot.y) * 180 / numpy.pi

    turnError = turnAngle(targetAngle,robot.theta)
    
    v = max(0,numpy.cos(turnError))*linearError/2
    v *= velocityKp
    w = omegaKp * turnError # - cause of pygame

    robot.prevDiff= omegaKd*(turnError-robot.prevTurnError)/dt #+ (omegaTau-dt)/(omegaTau+dt)*robot.prevDiff:
    w+=robot.prevDiff

    robot.target = karot
    
    w = numpy.clip(w, -numpy.pi/2,numpy.pi/2)

    robot.prevTurnError = turnError
    
    return((2*v - w*robot.width)/2, (2*v + w*robot.width)/2)

def Intersect(pos: tuple[float,float], pt1: tuple[float,float], pt2: tuple[float,float], lookAheadDis: float):
    cy,cx = pos[0]-pt1[0],pos[1]-pt1[1]
    
    py,px = pt2[0]-pt1[0],pt2[1]-pt1[1]
    
    a = px**2 + py**2
    b = px*cx + py*cy
    b *= 2
    c = cx**2 + cy**2 - lookAheadDis**2
    
    d = b**2-4*a*c
    if(d>=0):
        d = numpy.sqrt(d)
        t1 = (b + d)/(2*a)
        if(t1 > 1):
            return -2
        return (py*t1+pt1[0], px*t1 + pt1[1])

    return -1



## RenderFunction ##
def WriteToFB():
    __fb__.blit(__bg__,(0,0))
    for x in FieldObjects: x.draw(__fb__) 
    return

## Robots ##

friendBOT = RoboT((WINDOW_SIZE[0]/2,WINDOW_SIZE[1]/2), BotPic1 , 23.74)
BotList = list([friendBOT])
FieldObjects = list([friendBOT])    


# LOOKAHEAD POSITION COORDINATES
x_lookahead = None
y_lookahead = None
theta_lookahead = None

def catchall_new_lookahead(lookahead_postion):
    global x_lookahead, y_lookahead, theta_lookahead
    x_lookahead, y_lookahead, theta_lookahead = lookahead_postion

def pure_pursuit(iface):
    time.sleep(0.5) # Time to setup Glib mainloop
    
    global friendBOT,BotList,FieldObjects
    PAUSED = 1
    RUNNING = 2
    SELECT = 3
    OBSTACLE = 4
    WAYPOINTS = 5
    ROBOT = 6


    state = PAUSED

    clickMode = SELECT
    dt = 0
    lasttime = (pygame.time.get_ticks())
    selRobot = None
    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                exit()
            elif event.type == MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                if clickMode == SELECT:
                    addingWaypoint = True
                    for x in BotList:
                        if x.PtIsInside(pos): 
                            selRobot = x
                            addingWaypoint = False
                    if addingWaypoint and selRobot != None:
                        if event.button == 1:
                            if len(selRobot.waypoints) == 0:
                                selRobot.waypoints.append((selRobot.x,selRobot.y))
                            selRobot.waypoints.append(pos)
                        else:
                            if len(selRobot.waypoints)>0:
                                selRobot.waypoints.pop()
                                if len(selRobot.waypoints)-2<selRobot.lastWaypoint:
                                    selRobot.waypoints.clear()
                                    selRobot.lastWaypoint=0
                                    
                if  clickMode == OBSTACLE:
                    spos = tuple(pos)
                if clickMode == ROBOT:
                    nrobot = RoboT(pos, BotPic2, 21)
                    BotList.append(nrobot)
                    FieldObjects.append(nrobot)
            elif event.type == MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                if clickMode == OBSTACLE:
                    if spos != None:
                        FieldObjects.append( Obstacle((min(pos[0],spos[0]), min(pos[1],spos[1])), abs(pos[0]-spos[0]), abs(pos[1]-spos[1])))
            elif event.type == KEYDOWN:
                if event.key == K_SPACE:
                    if(state == RUNNING) : 
                        state = PAUSED
                        if not LOCALTESTING:
                            my_drive.axis0.controller.input_vel = 0
                            my_drive.axis1.controller.input_vel = 0
                    else : state = RUNNING
                if event.key == K_f:
                    clickMode = OBSTACLE
                if event.key == K_d:
                    clickMode = ROBOT
                if event.key == K_s:
                    clickMode = SELECT
                    
                
            
        dt = pygame.time.get_ticks()-lasttime
        lasttime+=dt
        dt/=1000
        
        if state == RUNNING:
            for x in BotList:
                if x==friendBOT: 
                    continue
                vl, vr = PurePursuit(x,dt)
                x.SetState(vl,vr)
                x.move(dt)
            vl, vr = PurePursuit(friendBOT,dt)
            if NO_ODOMETRY:
                friendBOT.SetState(vl,vr)
                friendBOT.move(dt)
            else:
                if LOCALTESTING:
                    ncords=iface.get_random_state_space()
                else:
                    ncords=iface.get_random_state_space()
                    my_drive.axis0.controller.input_vel = vl/friendBOT.cm2p/5
                    my_drive.axis1.controller.input_vel = vr/friendBOT.cm2p/5
                friendBOT.dmove(ncords[0]*friendBOT.cm2p+WINDOW_SIZE[0]/2, 
                            ncords[1]*friendBOT.cm2p+WINDOW_SIZE[1]/2, 
                            -ncords[2]+numpy.pi/2)
            

        WriteToFB()
                
        pygame.display.update()
        __clock__.tick(60)

def main():
    if NO_ODOMETRY:
        pure_pursuit(None)
        exit()
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()

    remote_object = bus.get_object("com.mgrobotics.Service",
                                   "/StateSpace")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    bus.add_signal_receiver(catchall_new_lookahead, dbus_interface = "com.mgrobotics.PurePursuit")

    # Define a thread that will concurrently run with mainloop
    # This thread will handle Pure Pursuit algorithm
    t = threading.Thread(target=pure_pursuit, args=(iface,))
    t.daemon = True
    t.start()

    # Define a mainloop that will handle dbus events
    mainloop = GLib.MainLoop()
    print("Running pure pursuit.")
    mainloop.run()

if __name__ == '__main__':
    main()

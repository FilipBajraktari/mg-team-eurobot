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

LOCALTESTING = False
NO_ODOMETRY = False

## Imports ##   
import tarfile
from tracemalloc import start
from xml.dom.expatbuilder import theDOMImplementation
import numpy, sys, pygame, math, cmath
from pygame.locals import *
from typing import List
from Robot import RoboT
from Obstacle import Obstacle
import Robot
import Obstacle
import time
import threading
import sys,os, copy
import glm as gm
import _rtrrt as rt
from glm import vec2

Robot.x0 = Obstacle.x0 = 1200/2
Robot.y0 = Obstacle.y0 = 800/2
Robot.cm2p = Obstacle.cm2p = 4

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

estop = False
state = PAUSED

ifaceLidar = None
#data = numpy.empty(WINDOW_SIZE[0]*WINDOW_SIZE[1]*4, dtype=numpy.int8)
#cairo_surface = cairo.ImageSurface.create_for_data(data, cairo.FORMAT_ARGB32, WINDOW_SIZE[0], WINDOW_SIZE[1],*4)

#ctx = cairo.Context(cairo_surface)

## Helper Classes ##

# TODO: Change wheel velocity to wheel rotational velocity

def catchall_estop():
    global estop
    estop = not estop
    return


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
    LOOKAHEAD=20
    karot = None
    if(len(robot.waypoints)==0):
        return 0,0
    for i in range(robot.lastWaypoint+1, len(robot.waypoints)):
        p = Intersect((robot.x,robot.y), robot.waypoints[robot.lastWaypoint], robot.waypoints[i],LOOKAHEAD)
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
    if(len(robot.waypoints)<=robot.lastWaypoint+1) and gm.distance2(robot.waypoints[1],(robot.x,robot.y))<=LOOKAHEAD**2+5:
        karot=robot.waypoints[-1]

    if(len(robot.waypoints)<=robot.lastWaypoint+1) and gm.distance2(robot.waypoints[-1],(robot.x,robot.y))<3:
        karot=None
        
    
    if(karot==None): 
        
        return (0,0)
    
    velocityKp = 6/2
    omegaKp = 4
    omegaKd = 0.1
    omegaTau = 0.1
    karot = vec2(karot)
    linearError = gm.distance(karot,vec2(robot.x,robot.y))

    localp = robot.toLocalSystem(karot)

    turnError = gm.atan(localp.y,localp.x)


    #w
    #:turnError = turnAngle(targetAngle,robot.theta)
    
    v = max(0,gm.cos(gm.clamp(turnError*1.2, -numpy.pi,numpy.pi)))*linearError/2
    v *= velocityKp
    w = omegaKp * turnError 

    robot.prevDiff= omegaKd*(turnError-robot.prevTurnError)/(dt+0.0001) + (omegaTau-dt)/(omegaTau+dt)*robot.prevDiff
    w+=robot.prevDiff

    
    w = numpy.clip(w, -numpy.pi/4,numpy.pi/4)

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
    global Map,friendBOT
    base = vec2(Robot.x0,Robot.y0)
    __fb__.blit(__bg__,(0,0))
    #xl = obstDistance(friendBOT,friendBOT,FieldObjects)
    #pygame.draw.circle(__fb__,(40,200,10),base+vec2(Robot.cm2p,-Robot.cm2p)*(friendBOT.x,friendBOT.y),xl*Robot.cm2p)
    for x in Map:
        if x[2]<0:
            continue
        y = Map[x[2]]
        pygame.draw.line(__fb__,(0,100,200),
        base+vec2(Robot.cm2p,-Robot.cm2p)*vec2((x[0]+250-1500)/10,(x[1]+250-1000)/10),
        base+vec2(Robot.cm2p,-Robot.cm2p)*vec2((y[0]+250-1500)/10,(y[1]+250-1000)/10),
        width=2)
    for x in (FieldObjects+oponents): x.draw(__fb__)
    
    return
## Robots ##

friendBOT = RoboT((0,0), BotPic1 , 23.74)
fBgoal = (50,50)
BotList = list([friendBOT])
FieldObjects = list([friendBOT])
Map = []


# LOOKAHEAD POSITION COORDINATES
x_lookahead = None
y_lookahead = None
theta_lookahead = None

state = 1

def catchall_new_lookahead(lookahead_postion):
    global friendBOT,Map
    if friendBOT==None:
        return
    friendBOT.waypoints = [(x[0]/10*friendBOT.cm2p,x[1]/10*friendBOT.cm2p) for x in lookahead_postion]
oponents = []

def pure_pursuit(iface):
    if not LOCALTESTING:
        import odrive
        my_drive = odrive.find_any()
    time.sleep(1) # Time to setup Glib mainloop
    
    global friendBOT,BotList,FieldObjects,fBgoal,state, ifaceLidar,oponents
    PAUSED = 1
    RUNNING = 2
    SELECT = 3
    OBSTACLE = 4
    WAYPOINTS = 5
    ROBOT = 6

    fBgoal = (-125.1,-75.1)

    state = PAUSED
    lasttime=0
    clickMode = SELECT
    dt = 0
    selRobot = None
    while True:
        if(not LOCALTESTING):
            oponentz = ifaceLidar.opponents_coordinates(1)
            oponents = [RoboT((x[0],[1]),BotPic2,23) for x in oponentz]
        if state != RUNNING or estop: 
            if not LOCALTESTING:
                my_drive.axis0.controller.input_vel = 0
                my_drive.axis1.controller.input_vel = 0
            friendBOT.vl,friendBOT.vr = (0,0)
        dt = time.time()-lasttime
        lasttime+=dt
        if state == RUNNING:
            for x in BotList:
                if x==friendBOT: 
                    continue
                vl, vr = PurePursuit(x,dt)
                x.SetState(vl,vr)
                x.move(dt)
                #state = PAUSED
            #vl,vr = PurePursuit(friendBOT,dt)
            goal = None
            for wa in friendBOT.waypoints:
                goal = wa
                if gm.distance2(wa,(friendBOT.x,friendBOT.y))>min(400,obstDistance(friendBOT,friendBOT,None)**2):
                    break
            if goal!=None and (gm.distance2(friendBOT.waypoints[-1],(friendBOT.x,friendBOT.y))<400):
                goal = friendBOT.waypoints[-1]
            if goal != None:
                DWA(goal,dt)
            #friendBOT.SetState(vl,vr)
            if NO_ODOMETRY:
                friendBOT.move(dt)
            else:
                if LOCALTESTING:
                    ncords=iface.get_state_space()
                else:   
                    ncords=iface.get_state_space()
                    f1 = time.time()
                    my_drive.axis0.controller.input_vel = -friendBOT.vl/(8*numpy.pi)*6
                    my_drive.axis1.controller.input_vel = friendBOT.vr/(8*numpy.pi)*6
                    #print(f1-time.time())
                friendBOT.dmove(ncords[0], ncords[1], ncords[2])
        
            
        print(1/dt)


def rrtSend():
    Obstacles = [(x.x * 10-250+1500,x.y * 10-250+1000,x.radius * 10) for x in (FieldObjects + oponents) if x!=friendBOT]
    if friendBOT == None: 
        return(1500-250,1000-250, fBgoal[0]*10+1500-250,fBgoal[1]*10+1000-250,[],0)
    return(friendBOT.x*10-250+1500,friendBOT.y*10-250+1000, fBgoal[0]*10-250+1500,fBgoal[1]*10-250+1000,Obstacles,len(Obstacles))

def rrtRecv(path,Tree):
    global friendBOT,Map
    if friendBOT==None:
        return
    friendBOT.waypoints = [((x[0]+250-1500)/10,(x[1]+250-1000)/10) for x in path]
    Map = Tree
    friendBOT.lastWaypoint=0

def rrtError(e):
    print(e)

def thePyGameThread():
    global friendBOT,BotList,FieldObjects,fBgoal, state
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
                pos = (vec2(pos[0],pos[1])-vec2(Robot.x0,Robot.y0))/vec2(Robot.cm2p,-Robot.cm2p)
                if clickMode == SELECT:
                    addingWaypoint = True
                    for x in BotList:
                        if x.PtIsInside(pos): 
                            selRobot = x
                            addingWaypoint = False
                    if addingWaypoint and selRobot != None:
                        if event.button == 1:
                            if(selRobot==friendBOT):
                                fBgoal=pos
                            else:
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
                    spos = pos
                if clickMode == ROBOT:
                    nrobot = RoboT(pos, BotPic2, 21)
                    BotList.append(nrobot)
                    FieldObjects.append(nrobot)
            elif event.type == MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                pos = (vec2(pos[0],pos[1])-vec2(Robot.x0,Robot.y0))/vec2(Robot.cm2p,-Robot.cm2p)
                if clickMode == OBSTACLE:
                    if spos != None:
                        FieldObjects.append( Obstacle.Obstacle((gm.min(pos,spos).x,gm.max(pos,spos).y), gm.abs(pos-spos).x, gm.abs(pos-spos).y))
            elif event.type == KEYDOWN:
                if event.key == K_SPACE:
                    if(state == RUNNING) : 
                        state = PAUSED
                    else : state = RUNNING
                if event.key == K_f:
                    clickMode = OBSTACLE
                if event.key == K_d:
                    clickMode = ROBOT
                if event.key == K_s:
                    clickMode = SELECT
        WriteToFB()
        pygame.display.update()
        __clock__.tick(60)

def DWA(GoalPoint: gm.vec2,dt):
    global friendBOT, BotList, FieldObjects
    MaxSpeed=30
    MaxTheta=numpy.deg2rad(40)
    MaxAcceleration=40
    
    GoalMultiplier=8
    
    ObstacleMultiplier = 6666
    ObstacleRoughMultiplier = 1900
    ObstDistMultiplier = 900
    
    VelocityMultiplier=0
    p =friendBOT.toLocalSystem(GoalPoint)
    heading =gm.atan(p.y, p.x)
    
    HeadingMultiplier = 90
    SAFEDISTANCE=15
    vL = friendBOT.vl
    vR = friendBOT.vr
    #print(time.time()-self.t)
    Steps = 0.8/dt
    
    a = 2*MaxAcceleration/5
    vLposiblearray = [vL-MaxAcceleration*dt+a*dt*i for i in range(0,5)]
    vLposiblearray.append(vL)
    vRposiblearray = [vR-MaxAcceleration*dt+a*dt*i for i in range(0,5)]
    vRposiblearray.append(vR)
    BestCost = 1000000000
    Best = None
    
    oldObstDist  = obstDistance(friendBOT,friendBOT,FieldObjects)
    for vLposible in vLposiblearray:
        for vRposible in vRposiblearray:
            if(abs(vLposible)<=MaxSpeed and abs(vRposible)<=MaxSpeed and abs((vLposible-vRposible)/friendBOT.width) <= MaxTheta ):
                predictState = predictPos(vLposible,vRposible,dt*Steps,friendBOT)

                p1 = predictState.toLocalSystem(GoalPoint)
                headingNew =gm.atan(p1.y, p1.x)
                


                x = friendBOT.x
                y = friendBOT.y
                A = gm.vec2(x,y)
                B = gm.vec2(predictState.x,predictState.y)
                DistImprovement = gm.distance(B,GoalPoint) - gm.distance(A,GoalPoint)
                obstacleDist = obstDistance(predictState,friendBOT,FieldObjects)
                obstacleRoughDist = obstRoughDistance(predictState,friendBOT,FieldObjects)
                headingImprovment = abs(headingNew) - abs(heading)

                DistCost = GoalMultiplier * DistImprovement
                headingCost = headingImprovment * HeadingMultiplier
                
                if(obstacleDist < max(1,(vLposible+vRposible))/MaxAcceleration):
                    obstacleRoughCost = ObstacleRoughMultiplier*max(1-obstacleRoughDist,(max(abs(vLposible),abs(vRposible))/MaxAcceleration-obstacleRoughDist))
                    obstacleCost = ObstacleMultiplier*max(1-obstacleRoughDist,(max(abs(vLposible),abs(vRposible))/MaxAcceleration-obstacleDist))
                else:
                    obstacleRoughCost = 0.0
                    obstacleCost = 0.0
                #VelCost = VelocityMultiplier*abs((vL+vR)/2 - TargetVel) 
                Cost = obstacleCost + DistCost + obstacleRoughCost + headingCost
                if(BestCost > Cost):
                    BestCost = Cost
                    Best = vLposible,vRposible
                    friendBOT.target = (B.x,B.y)

    friendBOT.vl, friendBOT.vr = Best


                

def predictPos(vL, vR, delta,friendBOT):
    p = copy.copy(friendBOT)
    p.vl=vL
    p.vr=vR
    p.move(delta)
    return p

def obstRoughDistance(prediction : RoboT,fiendBOT: RoboT,FielldObjects):
    global friendBOT,FieldObjects
    mindist = 100000
    for FO in (FieldObjects+oponents):
        if (FO.x,FO.y) != (friendBOT.x,friendBOT.y):
            x = prediction.toLocalSystem((FO.x,FO.y))
            p = x
            mindist = min(mindist,  (gm.length(p)-(FO.radius+1)-25))
    fb = vec2(prediction.x,prediction.y)
    a = (1500,1000) - abs(fb)-25
    mindist = min(mindist,  min(a.x,a.y))
    return mindist
def obstDistance(prediction : RoboT,fiendBOT: RoboT,FielldObjects):
    global friendBOT,FieldObjects
    mindist = 100000
    for FO in (FieldObjects+oponents):
        if (FO.x,FO.y) != (friendBOT.x,friendBOT.y):
            x = prediction.toLocalSystem((FO.x,FO.y))
            x.x += 6-23.5/2
            b = gm.vec2(23.5/2,35/2)
            p = x
            d = abs(p)-b
            mindist = min(mindist,  (gm.length(gm.max(d,0)) + min(max(d.x,d.y),0.0))-(FO.radius+1))
    fb = vec2(prediction.x,prediction.y)
    bx = [(-6,-17.5), (23.5-6,-17.5), (-6,17.5),(23.5-6,17.5)]
    bx = [fb+gm.rotate(x,prediction.theta) for x in bx]
    for p in bx:
        b = (1500,1000)
        d = abs(p) - b 
        mindist = min(mindist,  -(gm.length(gm.max(d,0)) + min(max(d.x,d.y),0.0)))
    return mindist

    

def RtRRT():
    time.sleep(2)
    print("started rrt")
    rt.startRRT(rrtSend, rrtRecv, rrtError)

def main():
    global ifaceLidar
    tr = threading.Thread(target=RtRRT)
    tr.daemon = True
    tr.start()
    trp = threading.Thread(target=thePyGameThread)
    trp.daemon = True
    trp.start()
    if NO_ODOMETRY:
        pure_pursuit(None)
        exit()
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus()

    remote_object = bus.get_object("com.mgrobotics.Service",
                                   "/StateSpace")
    lidar_object = bus.get_object("com.mgrobotics.LidarService", "/Lidar")
    iface = dbus.Interface(remote_object, "com.mgrobotics.OdometryInterface")
    ifaceLidar = dbus.Interface(lidar_object, "com.mgrobotics.LidarInterface")
    bus.add_signal_receiver(catchall_new_lookahead, dbus_interface = "com.mgrobotics.PurePursuit")
    bus.add_signal_receiver(catchall_estop, dbus_interface = "com.mgrobotics.EmergencyStop")

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

from abc import ABC, abstractmethod
import dbus
import glm
from glm import vec2
from PurePursuitTest.Robot import RoboT
import numpy
import copy
import time
import numpy as np
import RPi.GPIO as GPIO
import serial

# ODrive variables
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP    = 2

class Controller(ABC):
    iface      : dbus.Interface
    ifaceAI    : dbus.Interface
    ifaceRrt   : dbus.Interface
    ifaceLidar : dbus.Interface
    odrv0      : object
    Error      : str = None
    Complete   : bool = False

    wheel_distance : float = 0.235
    wheel_radius   : float = 0.084

    def __init__(self, iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0) -> None:
        self.iface = iface
        self.ifaceAI = ifaceAI
        self.ifaceRrt = ifaceRRT
        self.ifaceLidar = ifaceLidar
        self.odrv0 = odrv0

    @abstractmethod
    def Loop(self) -> None:
        pass
    @abstractmethod
    def SafeExit(self) -> None:
        pass

    def ControlLoop(self, CancelRequired : bool) -> None:
        if CancelRequired:
            self.SafeExit()
            self.Complete=True
        else:
            self.Loop()

    def rotate(self, omega):
        self.odrv0.axis0.controller.input_vel = omega*self.wheel_distance/self.wheel_radius
        self.odrv0.axis1.controller.input_vel = omega*self.wheel_distance/self.wheel_radius

    def move(self, velocity):
        self.odrv0.axis0.controller.input_vel = -2*velocity/self.wheel_radius
        self.odrv0.axis1.controller.input_vel = 2*velocity/self.wheel_radius
    
    def stop(self):
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.controller.input_vel = 0
        self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self.odrv0.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

class CommandCakeThing(Controller):
    ser : serial.Serial
    t = -1
    p =""
    strc:str
    sec = 2

    def __init__(self, iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0, ser, strc,sec) -> None:
        super().__init__(iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0)
        self.ser = ser
        self.strc = strc
        self.sec = sec

    def Loop(self) -> None:
        if (self.t == -1):
            self.t = time.time()
            self.ser.write(bytearray(self.strc, 'ascii'))
        if time.time()-self.t>0.1:
            if self.ser.inWaiting():
                self.p+=self.ser.read().decode('ascii')
            else:
                print(self.p)
        if time.time() - self.t>= self.sec:
            self.Complete = True
        return
    
    def SafeExit(self) -> None:
        time.sleep(0.1)
        print("Stopped due to canceling behaviour.")
        return
class TurnRelative(Controller):
    rel_pos : float
    goal    : float = None
    K_p     : float = 0.5

    ERROR_MARGINE : float = 0.1

    def __init__(self, iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0, rel_pos) -> None:
        super().__init__(iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0)
        self.rel_pos = rel_pos

    def Loop(self) -> None:
        x, y, theta, _, _ = self.iface.get_state_space()

        # Set the goal
        if self.goal == None:
            self.goal = theta + self.rel_pos
            self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP

        # Check if the goal is reached
        e = self.goal - theta
        if np.abs(e) < self.ERROR_MARGINE:
            self.stop()
            self.Complete = True
            return

        # Calculate omega using PID
        omega = self.K_p*e

        # Set omega
        self.rotate(omega)

        return
    
    def SafeExit(self) -> None:
        self.stop()
        print("Stopped due to canceling behaviour.")
        return

class Start(Controller):
    
    def __init__(self, iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0) -> None:
        super().__init__(iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def Loop(self) -> None:
        
        if GPIO.input(16) == GPIO.LOW:
            self.Complete=True
            self.SafeExit()
        return
    
    def SafeExit(self) -> None:
        print("Started run")
        return

class MoveRelative(Controller):
    distance : float
    goal     : float = None
    K_p      : float = 0.01
    direc :float
    sped : float
    t : float

    MAX_SPEED     : float = 0.1
    MAX_ACEL      : float = 0.1
    ERROR_MARGINE : float = 1

    def __init__(self, iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0, distance) -> None:
        super().__init__(iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0)
        self.distance = distance

    def Loop(self) -> None:
        x, y, theta, _, _ = self.iface.get_state_space()

        # Set the goal
        if self.goal == None:
            self.goal = (x + self.distance*np.cos(theta),
                         y + self.distance*np.sin(theta))
            self.t = time.time()
            self.sped = 0
            
            #self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            #self.odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        print(self.goal)
        print(x,y,theta)
        # Check if the goal is reached
        e = np.sqrt(np.square(self.goal[0]-x) + np.square(self.goal[1]-y))
       
        if np.abs(e) < self.ERROR_MARGINE:
            self.stop()
            self.Complete = True
            return

        # Calculate velocity using PID
        velocity = np.sign(self.distance)*self.K_p*e
        # velocity = np.minimum(velocity, self.MAX_SPEED)
        self.sped = np.sign(self.distance) * min(abs(self.sped)+self.MAX_ACEL*(min(time.time()-self.t,0.1)),min(self.MAX_SPEED,abs(velocity)))
        self.t=time.time()
        # Set velocity
        self.move(self.sped)

        return
    
    def SafeExit(self) -> None:
        self.stop()
        print("Stopped due to canceling behaviour.")
        return

class Traverse(Controller):
    GetTargetFunc=None
    Arg=None
    friendBOT : RoboT
    Timeout = 270

    def __init__(self, iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0, args, GetTrargetFunc) -> None:
        super().__init__(iface, ifaceAI, ifaceRRT, ifaceLidar, odrv0)
        self.GetTargetFunc = GetTrargetFunc
        self.Args = args
        self.friendBOT = RoboT((0,0),None,23.5)
        self.t = None
        self.ifaceRrt.SetDesiredPosition((-125.1,-75.1))

    def Loop(self) -> None:
        if not self.t:
            self.t=time.time()
        state = self.iface.get_state_space()
        self.obstacles = [vec2(x)/10 for x in self.ifaceLidar.opponents_coordinates(1)]
        #print(self.obstacles)
        self.friendBOT.dmove(state[0]-24, state[1], state[2])
        Target : vec2 = self.GetTargetFunc(self)
        self.ifaceRrt.SetDesiredPosition((Target.x,Target.y))
        waypoints = self.ifaceRrt.GetWaypoints()
        if len(waypoints)<1 or  round(waypoints[-1][0],3) != round(Target.x,3) or round(waypoints[-1][1], 3) != round(Target.y,3):
            if self.Timeout>0:
                self.Timeout-=1
                return
            else:
                self.Error = "Finding path to goal timed out"
                self.Complete = True
                self.SafeExit()
                return
        self.Timeout = 90
        goal = None
        for wa in waypoints:
            goal = wa
            if glm.distance2(wa,(self.friendBOT.x,self.friendBOT.y))>min(400,self.obstDistance(self.friendBOT,self.friendBOT,self.obstacles)**2):
                break
        if goal!=None and (glm.distance2(waypoints[-1],(self.friendBOT.x,self.friendBOT.y))<400):
            goal = waypoints[-1]
        if goal != None:
            self.DWA(self.obstacles, goal)
            self.odrv0.axis0.controller.input_vel = -self.friendBOT.vl/(8*numpy.pi)*6
            self.odrv0.axis1.controller.input_vel = self.friendBOT.vr/(8*numpy.pi)*6
        if glm.distance2(Target,(self.friendBOT.x,self.friendBOT.y))<10:
            self.SafeExit()
            self.Complete = True

    def SafeExit(self) -> None:
        self.odrv0.axis0.controller.input_vel= 0
        self.odrv0.axis1.controller.input_vel= 0
    def DWA(self, FieldObjects, GoalPoint: glm.vec2):
        MaxSpeed=30
        MaxTheta=numpy.deg2rad(40)
        MaxAcceleration=40
        
        GoalMultiplier=8
        
        ObstacleMultiplier = 6666
        ObstacleRoughMultiplier = 1900
        ObstDistMultiplier = 900
        
        VelocityMultiplier=0
        p = self.friendBOT.toLocalSystem(GoalPoint)
        heading =glm.atan(p.y, p.x)
        
        HeadingMultiplier = 90
        SAFEDISTANCE=15
        vL = self.friendBOT.vl
        vR = self.friendBOT.vr
        dt = time.time()-self.t
        #print(time.time()-self.t)
        self.t = time.time()
        Steps = 0.8/dt
        
        a = 2*MaxAcceleration/5
        vLposiblearray = [vL-MaxAcceleration*dt+a*dt*i for i in range(0,5)]
        vLposiblearray.append(vL)
        vRposiblearray = [vR-MaxAcceleration*dt+a*dt*i for i in range(0,5)]
        vRposiblearray.append(vR)
        BestCost = 1000000000
        Best = None
        
        oldObstDist  = self.obstDistance(self.friendBOT,self.friendBOT,FieldObjects)
        for vLposible in vLposiblearray:
            for vRposible in vRposiblearray:
                if(abs(vLposible)<=MaxSpeed and abs(vRposible)<=MaxSpeed and abs((vLposible-vRposible)/self.friendBOT.width) <= MaxTheta ):
                    predictState = self.predictPos(vLposible,vRposible,dt*Steps,self.friendBOT)

                    p1 = predictState.toLocalSystem(GoalPoint)
                    headingNew =glm.atan(p1.y, p1.x)
                    


                    x = self.friendBOT.x
                    y = self.friendBOT.y
                    A = glm.vec2(x,y)
                    B = glm.vec2(predictState.x,predictState.y)
                    DistImprovement = glm.distance(B,GoalPoint) - glm.distance(A,GoalPoint)
                    obstacleDist = self.obstDistance(predictState,self.friendBOT,FieldObjects)
                    obstacleRoughDist = self.obstRoughDistance(predictState,self.friendBOT,FieldObjects)
                    headingImprovment = abs(headingNew) - abs(heading)

                    DistCost = GoalMultiplier * DistImprovement
                    headingCost = headingImprovment * HeadingMultiplier
                    
                    if(obstacleDist < max(1,2*max(abs(vLposible),abs(vRposible))/MaxAcceleration)):
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
                        self.friendBOT.target = (B.x,B.y)

        self.friendBOT.vl, self.friendBOT.vr = Best

    def predictPos(self,vL, vR, delta,friendBOT):
        p = copy.copy(friendBOT)
        p.vl=vL
        p.vr=vR
        p.move(delta)
        return p

    def obstRoughDistance(self,prediction : RoboT,friendBOT: RoboT,FieldObjects):
        mindist = 100000
        for FO in FieldObjects:
            xx,yy = FO
            FO = glm.vec3(xx,yy,28)
            if (FO.x,FO.y) != (friendBOT.x,friendBOT.y):
                x = prediction.toLocalSystem((FO.x,FO.y))
                p = x
                mindist = min(mindist,  (glm.length(p)-FO.z-25))
        return mindist
    def obstDistance(self,prediction : RoboT,friendBOT: RoboT,FieldObjects):
        mindist = 100000
        for FO in FieldObjects:
            xx,yy = FO
            FO = glm.vec3(xx,yy,28)
            if (FO.x,FO.y) != (friendBOT.x,friendBOT.y):
                x = prediction.toLocalSystem((FO.x,FO.y))
                x.x += 6-23.5/2
                b = glm.vec2(23.5/2,35/2)
                p = x
                d = abs(p)-b
                mindist = min(mindist,  (glm.length(glm.max(d,0)) + min(max(d.x,d.y),0.0))-FO.z)
        DEBUG = True
        if DEBUG:
            fb = vec2(friendBOT.x,friendBOT.y)
            bx = [(-6,-17.5), (23.5-6,-17.5), (-6,17.5),(23.5-6,17.5)]
            bx = [fb+glm.rotate(x,prediction.theta) for x in bx]
            for p in bx:
                b = (1500,1000)
                d = abs(p) - b 
                mindist = min(mindist,  -(glm.length(glm.max(d,0)) + min(max(d.x,d.y),0.0)))
        return mindist

class Template_Controller(Controller):
    def Loop(self) -> None:
        Contemp = self.iface.get_state_space()
        print(f"{Contemp[0]}, {Contemp[1]}, {Contemp[2]}")
        if Contemp[2] > 1:
            self.Complete=True
        return
    def SafeExit(self) -> None:
        print("No pls")
        return
    
def TargetCake(self:Traverse)->vec2:
    a : vec2 = self.Args[0]
    fieldObjects = self.obstacles
    base = vec2(0,25+6)
    Samples = [a+glm.rotate(base,glm.radians(45*i)) for i in range(0,8)]
    minDist = 1000000000
    mini = None
    for sample in Samples:
        Free  = True
        for fo in fieldObjects:
            
            if(glm.distance2(sample,(fo[0],fo[1])))<24**2:
                Free=False
        Dist = glm.distance2(sample, (self.friendBOT.x,self.friendBOT.y))
        if Free and Dist<minDist:
            minDist = Dist
            mini = sample
    return mini

def TargetExact(self:Traverse)->vec2:
    a : vec2 = self.Args
    return a


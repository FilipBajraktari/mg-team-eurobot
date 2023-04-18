from abc import ABC, abstractmethod
import dbus
import glm
from glm import vec2
from PurePursuitTest.Robot import RoboT
import numpy
import copy
import time
import numpy as np

# ODrive variables
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP    = 2

class Controller(ABC):
    iface    : dbus.Interface
    ifaceAI  : dbus.Interface
    odrv0    : object
    Error    : str = None
    Complete : bool = False

    wheel_distance : float = 0.235
    wheel_radius   : float = 0.084

    def __init__(self, iface, ifaceAI, odrv0) -> None:
        self.iface = iface
        self.ifaceAI = ifaceAI
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


class TurnRelative(Controller):
    rel_pos : float
    goal    : float = None
    K_p     : float = 0.5

    ERROR_MARGINE : float = 0.1

    def __init__(self, iface, ifaceAI, odrv0, rel_pos) -> None:
        super().__init__(iface, ifaceAI, odrv0)
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

class MoveRelative(Controller):
    distance : float
    goal     : float = None
    K_p      : float = 0.01

    MAX_SPEED     : float = 0.5
    ERROR_MARGINE : float = 1

    def __init__(self, iface, ifaceAI, odrv0, distance) -> None:
        super().__init__(iface, ifaceAI, odrv0)
        self.distance = distance

    def Loop(self) -> None:
        x, y, theta, _, _ = self.iface.get_state_space()

        # Set the goal
        if self.goal == None:
            self.goal = (x + self.distance*np.cos(theta),
                         y + self.distance*np.sin(theta))
            self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP

        # Check if the goal is reached
        e = np.sqrt(np.square(self.goal[0]-x) + np.square(self.goal[1]-y))
        # print(self.goal)
        # print(x, y)
        # print(e)
        # print("----------------------")
        if np.abs(e) < self.ERROR_MARGINE:
            self.stop()
            self.Complete = True
            return

        # Calculate velocity using PID
        velocity = np.sign(self.distance)*self.K_p*e
        # velocity = np.minimum(velocity, self.MAX_SPEED)

        # Set velocity
        self.move(velocity)

        return
    
    def SafeExit(self) -> None:
        self.stop()
        print("Stopped due to canceling behaviour.")
        return

class Traverse(Controller):
    GetTargetFunc : function
    Args : function
    friendBot : RoboT
    def __init__(self, iface, ifaceAI, odrv0, goal, args, GetTrargetFunc : function) -> None:
        self.GetTargetFunc = GetTrargetFunc
        self.Args = args
        self.friendBot = RoboT((0,0),None,23.5)
        self.goal = goal

    def Loop(self) -> None:
        state = self.iface.get_state_space()
        self.obstacles = self.iface.get_obstacles()
        self.friendBot.dmove(state)
        Target : vec2 = self.GetTargetFunc(self.Args)
        self.ifaceAI.emit_new_lookahead(Target.x,Target.y)
        self.DWA(self.obstacles,Target)
        self.odrv0.axis0.controller.input_value=  self.friendBOT.vl
        self.odrv0.axis1.controller.input_value= -self.friendBOT.vr

    def SafeExit(self) -> None:
        self.odrv0.axis0.controller.input_value= 0
        self.odrv0.axis1.controller.input_value= 0
    
    def DWA(self, FieldObjects, GoalPoint: glm.vec2):
        MaxSpeed=30
        MaxTheta=numpy.deg2rad(40)
        MaxAcceleration=50
        #MaxTurnAcceleration=numpy.deg2rad(90)
        GoalMultiplier=8
        
        ObstacleMultiplier = 6666
        ObstacleRoughMultiplier = 1900
        ObstDistMultiplier = 900
        #TargetVel = 12 * gm.length(gm.vec2(friendBOT.x,friendBOT.y)-GoalPoint)
        VelocityMultiplier=0
        p = self.friendBOT.toLocalSystem(GoalPoint)
        heading =glm.atan(p.y, p.x)
        
        HeadingMultiplier = 90
        SAFEDISTANCE=15
        vL = self.friendBOT.vl
        vR = self.friendBOT.vr
        dt = 0.1
        Steps = 8
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

    def predictPos(vL, vR, delta,friendBOT):
        p = copy.copy(friendBOT)
        p.vl=vL
        p.vr=vR
        p.move(delta)
        return p

    def obstRoughDistance(prediction : RoboT,friendBOT: RoboT,FieldObjects):
        mindist = 100000
        for FO in FieldObjects:
            if (FO.x,FO.y) != (friendBOT.x,friendBOT.y):
                x = prediction.toLocalSystem((FO.x,FO.y))
                p = x
                mindist = min(mindist,  (glm.length(p)-FO.radius-25))
        return mindist
    def obstDistance(prediction : RoboT,friendBOT: RoboT,FieldObjects):
        mindist = 100000
        for FO in FieldObjects:
            if (FO.x,FO.y) != (friendBOT.x,friendBOT.y):
                x = prediction.toLocalSystem((FO.x,FO.y))
                x.x += 6-23.5/2
                b = glm.vec2(23.5/2,35/2)
                p = x
                d = abs(p)-b
                mindist = min(mindist,  (glm.length(glm.max(d,0)) + min(max(d.x,d.y),0.0))-FO.radius)
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
    
def TargetCake()->vec2:
    a : vec2
    fieldObjects:list(tuple[float,float,float])
    
    base = vec2(0,18+6)
    Samples = [a+glm.rotate(base,glm.radians(45*i)) for i in range(0,8)]
    minDist = 1000000000
    mini = None
    for sample in Samples:
        for fo in fieldObjects:
            if(glm.length2(sample,(fo[0],fo[1])))>24**2 and glm.length2()

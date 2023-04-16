import pygame
import math
import copy
import glm
from glm import vec2
#from pygame import *

x0,y0 = 0,0
cm2p = 4
class RoboT:
    def __init__(self,startpos: tuple[float, float], robotImg, width):
        self.cm2p=1200/300
        #Initial state
        self.width=width
        self.x=startpos[0]
        self.y=startpos[1]
        self.theta = math.radians(0)
        self.vl = 0
        self.vr = 0
        self.omega = 0
        self.maxspeed = 2
        self.waypoints = []
        self.lastWaypoint = 0
        self.target = None
        self.prevPos = []
        self.prevPosC = 0
        self.prevRot= math.radians(0)
        self.prevDiff=0
        self.prevTurnError=0
        self.radius = robotImg.get_width()/2/cm2p
        

        #Graphics
        self.img = robotImg
    
    def draw(self, map: pygame.Surface):
        base = vec2(x0,y0)
        m = vec2(cm2p,-cm2p)
        for x,y in zip(self.waypoints[:-1:],self.waypoints[1::]):
            a = vec2(x)
            b = vec2(y)
            a = base + m*a
            b = base + m*b
            pygame.draw.line(map,(0,0,0),a,b,3)
        for x in self.waypoints: pygame.draw.circle(map,(255,0,0) ,base+m*vec2(x),2)
        '''
        self.prevPos.append((self.x,self.y))
        if self.prevPosC>10000: self.prevPos.pop(0)
        else: self.prevPosC+=1
        for x,y in zip(self.prevPos[:-5:5],self.prevPos[5::5]):
            pygame.draw.line(map,(0,255,255),x,y,3)
        '''
        base = vec2(x0,y0)
        m = vec2(cm2p,-cm2p)
        self.roatated=pygame.transform.rotozoom(self.img,math.degrees(self.theta)+90,1)
        self.rect = self.roatated.get_rect(center = base + m*vec2(self.x,self.y))
        rot = copy.copy(self.roatated)
        rec =  copy.copy(self.rect)
        map.blit(rot, rec)
        #pygame.draw.circle(map, (255,255,0) ,(self.x,self.y), 22)

        if self.target != None : 
            self.target = vec2(self.target[0],self.target[1])
            self.target = base + m*self.target
            pygame.draw.circle(map, (0,255,0) ,self.target, 2)

    def move(self,dt : float):
        if(round(self.vr,3) == round(self.vl,3)):
            self.x+=self.vl*dt*math.cos(self.theta)
            self.y+=self.vr*dt*math.sin(self.theta)
        elif(round(self.vr,3) == -round(self.vl,3)):
            dtheta =(self.vr-self.vl)/self.width*dt
            self.theta += dtheta
        elif True:
            self.prevRot = self.theta
            dtheta =(self.vr-self.vl)/self.width*dt
            R = self.width/2 * (self.vr+self.vl) /(self.vr-self.vl)
            self.y-= R*(math.cos(self.theta+dtheta)-math.cos(self.theta))
            self.x+= R*(math.sin(self.theta+dtheta)-math.sin(self.theta))
            self.theta += dtheta
        else: 
            dtheta =(self.vr-self.vl)/self.width*dt
            self.theta += dtheta
        if(self.theta>math.pi): self.theta += -2*math.pi
        elif(self.theta<-math.pi): self.theta+= 2*math.pi
        
    
    def dmove(self, x: float, y: float, theta: float):
        self.prevRot = self.theta

        self.x=x
        self.y=y
        self.theta=theta
        if(self.theta>math.pi): self.theta += -2*math.pi
        elif(self.theta<-math.pi): self.theta+= 2*math.pi
        
        base = vec2(x0,y0)
        m = vec2(cm2p,-cm2p)
        self.roatated=pygame.transform.rotozoom(self.img,math.degrees(self.theta),1)
        self.rect = self.roatated.get_rect(center = base + m*vec2(self.x,self.y))
            
    def SetState(self, vl, vr):
        self.vl = vl
        self.vr = vr
    
    def PtIsInside(self, pos: vec2) -> bool :
        if glm.distance2(pos,(self.x,self.y))<=(self.width*self.width)/4:
            return True
        return False
    
    def toLocalSystem(self, pos: vec2)->vec2:
        a = (vec2(pos)-(self.x,self.y))
        return glm.rotate(a, -self.theta)
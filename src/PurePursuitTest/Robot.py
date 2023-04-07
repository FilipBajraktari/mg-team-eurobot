import pygame
import math
#from pygame import *


class RoboT:
    def __init__(self,startpos: tuple[float, float], robotImg, width):
        self.cm2p=1200/300
        #Initial state
        self.width=width*self.cm2p
        self.x=startpos[0]
        self.y=startpos[1]
        self.theta = math.radians(90)
        self.vl = 10*self.cm2p
        self.vr = 10*self.cm2p
        self.omega = 0
        self.maxspeed = 2*self.cm2p
        self.waypoints = []
        self.lastWaypoint = 0
        self.target = None
        self.prevPos = []
        self.prevPosC = 0
        self.prevRot= math.radians(90)
        self.prevDiff=0
        self.prevTurnError=0
        self.radius = robotImg.get_width()/2

        #Graphics
        self.img = robotImg
        self.roatated = self.img
        self.rect = self.roatated.get_rect(center = (self.x,self.y))
    
    def draw(self, map: pygame.Surface):
        for x,y in zip(self.waypoints[:-1:],self.waypoints[1::]):
            pygame.draw.line(map,(0,0,0),x,y,3)
        for x in self.waypoints: pygame.draw.circle(map,(255,0,0) ,x,2)
        self.prevPos.append((self.x,self.y))
        if self.prevPosC>10000: self.prevPos.pop(0)
        else: self.prevPosC+=1
        for x,y in zip(self.prevPos[:-5:5],self.prevPos[5::5]):
            pygame.draw.line(map,(0,255,255),x,y,3)

        map.blit(self.roatated, self.rect)
        

        if self.target != None : 
            self.target = int(self.target[0]),int(self.target[1])
            pygame.draw.circle(map, (0,255,0) ,self.target, 2)

    def move(self,dt : float):
        self.prevRot = self.theta

        self.y+=((self.vl+self.vr)/2)*math.cos(self.theta)*dt
        self.x+=((self.vl+self.vr)/2)*math.sin(self.theta)*dt
        self.theta+=(self.vr-self.vl)/self.width*dt
        
        if(self.theta>math.pi): self.theta += -2*math.pi
        elif(self.theta<-math.pi): self.theta+= 2*math.pi
        
        self.roatated=pygame.transform.rotozoom(self.img,math.degrees(self.theta),1)
        self.rect = self.roatated.get_rect(center = (self.x,self.y))
    
    def dmove(self, x: float, y: float, theta: float):
        self.prevRot = self.theta

        self.x=x
        self.y=y
        self.theta=theta
        if(self.theta>math.pi): self.theta += -2*math.pi
        elif(self.theta<-math.pi): self.theta+= 2*math.pi
        
        self.roatated=pygame.transform.rotozoom(self.img,math.degrees(self.theta),1)
        self.rect = self.roatated.get_rect(center = (self.x,self.y))
            
    def SetState(self, vl, vr):
        self.vl = vl
        self.vr = vr
    
    def PtIsInside(self, pos: tuple[float,float]) -> bool :
        x,y = pos
        if (x-self.x)*(x-self.x)+(y-self.y)*(y-self.y)<=(self.width*self.width)/4:
            return True
        return False
    
    def toLocalSystem(self, pos: tuple[float, float])->tuple[float,float]:
        x = (pos[0]-self.x)
        y = (pos[1]-self.y)
        
        return (x*math.cos(self.theta)+y*math.sin(self.theta), y*math.cos(self.theta)+x*math.sin(self.theta))
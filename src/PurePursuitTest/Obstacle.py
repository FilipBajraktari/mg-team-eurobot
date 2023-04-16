import pygame, math
from pygame.locals import *

x0,y0 = 0,0
cm2p = 4


class Obstacle:
    def __init__(self,startpos: tuple[float, float],  width, height):
        #Initial state
        self.x = startpos[0]+width/2
        self.y = startpos[1]-height/2
        self.xi = startpos[0]
        self.yi = startpos[1]
        self.height = height
        self.width=width
        self.radius = math.sqrt(height**2 + width**2)/2

    
    def draw(self, map: pygame.Surface):
        xi =x0 + self.xi*cm2p
        yi =y0 - self.yi*cm2p 


        width = self.width*cm2p
        height = self.width*cm2p
        pygame.draw.rect(map,(100,100,100),(xi,yi,width,height), 0)
        pygame.draw.rect(map,(0,0,0),(xi,yi,width,height), 4)
    
    def PtIsInside(self, pos: tuple[float,float]) -> bool :
        return self.x<pos[0]<self.x+self.width and self.y<pos[1]<self.y+self.height 
    
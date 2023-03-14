import pygame, math
from pygame.locals import *


class Obstacle:
    def __init__(self,startpos: tuple[float, float],  width, height):
        #Initial state
        self.x = startpos[0]
        self.y = startpos[1]
        self.height = height
        self.width=width

    
    def draw(self, map: pygame.Surface):
        pygame.draw.rect(map,(100,100,100),(self.x,self.y,self.width,self.height), 0)
        pygame.draw.rect(map,(0,0,0),(self.x,self.y,self.width,self.height), 4)
    
    def PtIsInside(self, pos: tuple[float,float]) -> bool :
        return self.x<pos[0]<self.x+self.width and self.y<pos[1]<self.y+self.height 
    
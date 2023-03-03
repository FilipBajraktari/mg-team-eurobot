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
        pygame.draw.rect(map,color=(100,100,100),rect=(self.x,self.y,self.width,self.height), width=0)
        pygame.draw.rect(map,color=(0,0,0),rect=(self.x,self.y,self.width,self.height), width=4)
    
    def PtIsInside(self, pos: tuple[float,float]) -> bool :
        return self.x<pos[0]<self.x+self.width and self.y<pos[1]<self.y+self.height 
    
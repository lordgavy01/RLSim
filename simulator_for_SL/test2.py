import numpy as np
from math import *
import pygame


np.random.seed(10)
pygame.init()

image_name='cir.png'

class Background(pygame.sprite.Sprite):
    def __init__(self, image_file, location):
        pygame.sprite.Sprite.__init__(self)  #call Sprite initializer
        self.image = pygame.image.load(image_file)
        self.rect = self.image.get_rect()
        self.rect.left, self.rect.top = location

Background=Background(image_name,[0,0])
screen=pygame.display.set_mode((Background.image.get_width(),Background.image.get_height()))
screen.blit(Background.image, Background.rect)
# screen.fill((255,255,255))
clicks=0
running=True
while running:
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
        if events.type==pygame.MOUSEBUTTONDOWN:
            pos=pygame.mouse.get_pos()
            clicks+=1
            print(pos)
            if clicks==1:
                A=pos
                pygame.draw.circle(Background.image,(255,0,0),A,5)
            if clicks==2:
                B=pos
                pygame.draw.circle(Background.image,(0,255,0),B,5)
    pygame.display.update()
    
                
from util import *
from random import randint

from lidar import *
from planner import *
from environment import *
from agent import *

initMap()
pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")

running=True
key=0
paused=False
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))

print("Map Dimensions:",(mapBackground.image.get_width(),mapBackground.image.get_height()))

env=Environment()
env.reset()

while running:
    screen.blit(mapBackground.image, mapBackground.rect)
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
            break

    user_input=pygame.key.get_pressed()
    if(user_input[pygame.K_UP] or user_input[pygame.K_w]):
        action=env.agentStates[0].selectAction()
        reward=env.executeAction(action)
    
    env.render(screen)
    pygame.display.update()
    key+=1

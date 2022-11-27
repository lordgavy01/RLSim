from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
from config import *

pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")
obstacles=initMap(mapObstaclesFilename="map_obstacles.txt")
mapBackground=getMapBackground(mapImageFilename="map.png")

running=True
key=0
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))
print("Map Dimensions:",(mapBackground.image.get_width(),mapBackground.image.get_height()))

env=Environment()
env.reset(obstacles=obstacles)

while running:
    screen.blit(mapBackground.image, mapBackground.rect)
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
            pygame.quit()
            break

    user_input=pygame.key.get_pressed()
    if(user_input[pygame.K_UP] or user_input[pygame.K_w] or key%20==0):
        action=env.agentStates[0].selectAction("NN")
        reward=env.executeAction(action,NOISE)
    
    env.render(screen)
    pygame.display.update()
    key+=1

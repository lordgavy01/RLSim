from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
from config import *

MAP_NAME="small_map"
APF_DATA_ITER=300
APF_DATA_NO_ROTATE_KEEP=0.4
USE_CHECKPOINT=True

pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")
obstacles=initMap(mapObstaclesFilename=f"Maps/{MAP_NAME}_obstacles.txt")
mapBackground=getMapBackground(mapImageFilename=f"Maps/{MAP_NAME}.png")

apfDataFilename = f"Datasets/apf_data_{MAP_NAME}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}.csv"
checkpointFilename= f"Checkpoints/checkpoint_{MAP_NAME}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}.pth"

policy=Policy()
env=Environment()
env.reset(obstacles=obstacles,agentSubGoals=AGENT_SUBGOALS)

running=True
key=0
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))
print("Map Dimensions:",(mapBackground.image.get_width(),mapBackground.image.get_height()))

if USE_CHECKPOINT:
    policy.loadWeights(checkpointFilename)
else:
    policy.storeLearntWeightsFromData(apfDataFilename,checkpointFilename)
    policy.loadWeights(checkpointFilename)

while running:
    screen.blit(mapBackground.image, mapBackground.rect)
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
            pygame.quit()
            break

    user_input=pygame.key.get_pressed()
    if(user_input[pygame.K_UP] or user_input[pygame.K_w] or key%1==0):
        action=env.agentStates[0].selectAction("NN",policy)
        # action=env.agentStates[0].selectAction("APF")
        reward=env.executeAction(action,noise=0.1,goalDistanceThreshold=6)
    
    env.render(screen)
    pygame.display.update()
    key+=1

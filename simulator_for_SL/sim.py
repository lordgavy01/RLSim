from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
from config import *
import time

MAP_NAME="map3"
APF_DATA_ITER=300
APF_DATA_NO_ROTATE_KEEP=0.4
USE_CHECKPOINT=True
GOAL_DISTANCE_THRESHOLD=6

pygame.init()
pygame.display.set_caption("APF")
obstacles=initMap(mapObstaclesFilename=f"Maps/{MAP_NAME}_obstacles.txt")
mapBackground=getMapBackground(mapImageFilename=f"Maps/{MAP_NAME}.png")

apfDataFilename = f"Datasets/apf_data_{MAP_NAME}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}.csv"
checkpointFilename= f"Checkpoints/checkpoint_{MAP_NAME}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}.pth"

policy=Policy()
env=Environment()
env.reset(obstacles=obstacles,agentSubGoals=AGENT_SUBGOALS5)

running=True
key=0
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))
print("Map Dimensions:",(mapBackground.image.get_width(),mapBackground.image.get_height()))
screen.blit(mapBackground.image, mapBackground.rect)
env.renderSubGoals(screen)
pygame.display.update()
time.sleep(10)

# if USE_CHECKPOINT:
#     policy.loadWeights(checkpointFilename)
# else:
#     policy.storeLearntWeightsFromData(apfDataFilename,checkpointFilename)
#     policy.loadWeights(checkpointFilename)

collisionFlag=False
numTimestamps=0
pathClearance=INF
sumGoalDistance=0
controlKey=0

while running:
    controlKey+=1
    screen.blit(mapBackground.image, mapBackground.rect)
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
            break
    
    if(controlKey%10==0):
        # action=env.agentStates[0].selectAction("NN",policy)
        action=env.agentStates[0].selectAction("APF")
        reward=env.executeAction(action,noise=0.1,goalDistanceThreshold=GOAL_DISTANCE_THRESHOLD)
        env.render(screen)
        pygame.display.update()
        
        numTimestamps+=1
        pathClearance=min(pathClearance,env.getAgentClearances()[0])
        goalDistance=euclidean((env.agentPoses[0][0],env.agentPoses[0][1]),(env.agentGoals[0][0],env.agentGoals[0][1]))
        sumGoalDistance+=goalDistance

        if(env.getAgentClearances()[0]==-1):
            print(env.getAgentClearances())
            print("Robot Collided!!!")
            collisionFlag=True
            break
        
        if goalDistance<GOAL_DISTANCE_THRESHOLD:
            if (env.agentProgress[0]+1)==(len(env.agentSubGoals[0])-1):
                print("Robot reached Goal!")
                running=False
                break
        
        if(numTimestamps>500):
            print("Time Limit Exceeded")
            collisionFlag=True
            break

if not collisionFlag:
    print()
    print(f"Number of timestamps: {numTimestamps}")
    print(f"Path Clearance: {pathClearance}")
    print(f"Average Goal Distance along Path: {sumGoalDistance/numTimestamps}")
from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
from config import *
import csv
import time
import matplotlib.pyplot as plt

MAP_NAME="map3"
APF_DATA_ITER=300
APF_DATA_NO_ROTATE_KEEP=0.4
USE_CHECKPOINT=True
GOAL_DISTANCE_THRESHOLD=6
AGENT_SUBGOALS=AGENT_SUBGOALS5

pygame.init()
obstacles=initMap(mapObstaclesFilename=f"Maps/{MAP_NAME}_obstacles.txt")
mapBackground=getMapBackground(mapImageFilename=f"Maps/{MAP_NAME}.png")
env=Environment()
env.reset(obstacles=obstacles,agentRadius=AGENT_RADIUS,agentSubGoals=AGENT_SUBGOALS)
pygame.display.set_caption(f"DAgger")
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))
screen.blit(mapBackground.image, mapBackground.rect)
env.renderSubGoals(screen)
pygame.display.update()
time.sleep(5)

apfDataFilename = f"Datasets/apf_data_{MAP_NAME}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}.csv"
checkpointFilename= f"Checkpoints/checkpoint_{MAP_NAME}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}.pth"
tempDataFilename = f"Datasets/iter_data_{MAP_NAME}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}.csv"
tempCheckpointFilename= f"Checkpoints/iter_checkpoint_{MAP_NAME}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}.pth"

fields=["output_linear_velocity","output_angular_velocity","distance_from_goal","angle_from_goal"]
fields+=[f"lidar_depth_{i}" for i in range(1,1+NUMBER_OF_LIDAR_ANGLES)]

with open(tempDataFilename,'w') as csvfile: 
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(fields)

pathNumTimestamps=[]
pathClearances=[]
pathAvgGoalDistances=[]

keepIterating=True
policy=Policy()
NUM_ITERATIONS=20
for i in range(1,1+NUM_ITERATIONS):  
    print(f"\n***Iteration {i}***")
    env=Environment()
    env.reset(obstacles=obstacles,agentRadius=AGENT_RADIUS,agentSubGoals=AGENT_SUBGOALS)
    pygame.display.set_caption(f"DAgger: Iteration {i}")
    screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))
    print("Initialized map.")

    if USE_CHECKPOINT:
        if i==1:
            policy.loadWeights(checkpointFilename)
        else:
            policy.storeLearntWeightsFromData(tempDataFilename,tempCheckpointFilename)
            policy.loadWeights(tempCheckpointFilename)
    else:
        if i==1:
            policy.storeLearntWeightsFromData(apfDataFilename,checkpointFilename)
            policy.loadWeights(checkpointFilename)
        else:
            policy.storeLearntWeightsFromData(tempDataFilename,tempCheckpointFilename)
            policy.loadWeights(tempCheckpointFilename)

    rows=[]
    running=True
    lastProgress=0
    isApfOn=0
    robotColor=(255,0,0)

    collisionFlag=False
    numTimestamps=0
    pathClearance=INF
    sumGoalDistance=0
    
    while running:
        screen.blit(mapBackground.image, mapBackground.rect)
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                running=False
                keepIterating=False
                break
            if event.type==pygame.KEYDOWN:
                key_name=pygame.key.name(event.key)
                key_name=key_name.upper()
                print(key_name)  
                if(key_name=="RETURN"):
                    isApfOn=1-isApfOn
                    if isApfOn==1:
                        robotColor=(0,0,0)
                    else:
                        robotColor=(255,0,0)

        if(isApfOn==0):
            action=env.agentStates[0].selectAction("NN",policy)
        else:
            action=env.agentStates[0].selectAction("APF")
        apfAction=env.agentStates[0].selectAction("APF")
        row=[apfAction[0],apfAction[1],
            env.agentStates[0].distanceGoal,
            env.agentStates[0].thetaGoal,
            ]+env.agentStates[0].lidarData[1]
        reward=env.executeAction(action,noise=0.1,goalDistanceThreshold=GOAL_DISTANCE_THRESHOLD)
        env.render(screen,robotColor)
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
        
        if(numTimestamps>400):
            print("Time Limit Exceeded")
            collisionFlag=True
            break

        if(abs(row[1])<abs(radians(1))):
            epsilon=random.uniform(0,1)
            if(epsilon<=0.4):
                rows.append(row)
        else:
            rows.append(row)

    if not keepIterating:
        break

    if not collisionFlag:
        print()
        print(f"Number of timestamps: {numTimestamps}")
        print(f"Path Clearance: {pathClearance}")
        print(f"Average Goal Distance along Path: {sumGoalDistance/numTimestamps}")
        print(f"Number of learning iterations: {i}")
        pathNumTimestamps.append(numTimestamps)
        pathClearances.append(pathClearance)
        pathAvgGoalDistances.append(sumGoalDistance/numTimestamps)
    else:
        pathNumTimestamps.append(INF)
        pathClearances.append(0)
        pathAvgGoalDistances.append(sumGoalDistance/numTimestamps)
    
    print(f"Adding {len(rows)} rows to database.")
    with open(tempDataFilename,'a') as csvfile: 
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(rows)

print()
print(pathNumTimestamps)
print(pathClearances)   
print(pathAvgGoalDistances)    
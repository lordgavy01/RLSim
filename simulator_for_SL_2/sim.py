from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
from config import *
import csv
import time
import matplotlib.pyplot as plt

start_time=time.time()

MAPS=["small_map","small_map2","map2","map3","map4","squares_map"]
SUBGOALS=[AGENT_SUBGOALS2,AGENT_SUBGOALS3,AGENT_SUBGOALS4,AGENT_SUBGOALS5,AGENT_SUBGOALS6,AGENT_SUBGOALS7]
APF_PARAMS=[APF_PARAMS_S,APF_PARAMS_S,APF_PARAMS_S,APF_PARAMS_S,APF_PARAMS_S,APF_PARAMS_S]
# APF_PARAMS=[APF_PARAMS_1,APF_PARAMS_2,APF_PARAMS_1,APF_PARAMS_1,APF_PARAMS_3,APF_PARAMS_4]

APF_DATA_ITER=150
APF_DATA_NO_ROTATE_KEEP=0.4
USE_CHECKPOINT=True
GOAL_DISTANCE_THRESHOLD=6
FILE_NUM=7

obstacles=[]
mapBackgrounds=[]
for map in MAPS:
    obstacles.append(initMap(mapObstaclesFilename=f"Maps/{map}_obstacles.txt"))
    mapBackgrounds.append(getMapBackground(mapImageFilename=f"Maps/{map}.png"))

env=Environment()
env.reset(obstacles=obstacles[0],agentRadius=AGENT_RADIUS,agentSubGoals=SUBGOALS[0])
pygame.init()
pygame.display.set_caption(f"DAgger")
screen=pygame.display.set_mode((mapBackgrounds[0].image.get_width(),mapBackgrounds[0].image.get_height()))
screen.blit(mapBackgrounds[0].image, mapBackgrounds[0].rect)
env.renderSubGoals(screen)
pygame.display.update()
# time.sleep(2)

# for j in range(len(MAPS)):
#     env=Environment()
#     env.reset(obstacles=obstacles[j],agentRadius=AGENT_RADIUS,agentSubGoals=SUBGOALS[j])
#     pygame.display.set_caption(f"Map {j}")
#     screen=pygame.display.set_mode((mapBackgrounds[j].image.get_width(),mapBackgrounds[j].image.get_height()))
#     screen.blit(mapBackgrounds[j].image, mapBackgrounds[j].rect)
#     env.renderSubGoals(screen)
#     pygame.display.update()
#     time.sleep(7)

pathNumTimestamps=[[] for _ in range(len(MAPS))]
pathClearances=[[] for _ in range(len(MAPS))]
pathAvgGoalDistances=[[] for _ in range(len(MAPS))]

keepIterating=True
policy=Policy()
curFile=f"WorkingCheckpoints/iter_checkpoint_{FILE_NUM}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}_8.pth"
policy.loadModel(curFile)
NUM_ITERATIONS=5
for i in range(1,1+NUM_ITERATIONS): 
    print(f"\n***Iteration {i}***")
    allMapsPassed=True

    rows=[]
    for j in range(len(MAPS)):
        print(f"\n*Map {j}*")
        env=Environment()
        env.reset(obstacles=obstacles[j],agentRadius=AGENT_RADIUS,agentSubGoals=SUBGOALS[j])
        pygame.display.set_caption(f"DAgger: Iteration {i} Map {j}")
        screen=pygame.display.set_mode((mapBackgrounds[j].image.get_width(),mapBackgrounds[j].image.get_height()))
        screen.blit(mapBackgrounds[j].image, mapBackgrounds[j].rect)
        running=True
        lastProgress=0
        isApfOn=1
        robotColor=(255,0,0)
        collisionFlag=False
        numTimestamps=0
        pathClearance=INF
        sumGoalDistance=0
    
        while running:
            screen.blit(mapBackgrounds[j].image, mapBackgrounds[j].rect)
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
                action=env.agentStates[0].selectAction("APF",apfParams=APF_PARAMS[j])
            apfAction=env.agentStates[0].selectAction("APF",apfParams=APF_PARAMS[j])
            row=[apfAction[0],apfAction[1],
                env.agentStates[0].distanceGoal,
                env.agentStates[0].thetaGoal,
                ]+env.agentStates[0].lidarData[1]
            reward=env.executeAction(action,noise=0.5,goalDistanceThreshold=GOAL_DISTANCE_THRESHOLD)
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

        if not keepIterating:
            break

        if not collisionFlag:
            print()
            print(f"Number of timestamps: {numTimestamps}")
            print(f"Path Clearance: {pathClearance}")
            print(f"Average Goal Distance along Path: {sumGoalDistance/numTimestamps}")
            print(f"Number of learning iterations: {i}")
            pathNumTimestamps[j].append(numTimestamps)
            pathClearances[j].append(pathClearance)
            pathAvgGoalDistances[j].append(sumGoalDistance/numTimestamps)
        else:
            allMapsPassed=False
            pathNumTimestamps[j].append(INF)
            pathClearances[j].append(0)
            pathAvgGoalDistances[j].append(sumGoalDistance/numTimestamps)
    
    if not keepIterating:
            break

    print("Execution Time since start:",(time.time()-start_time),"s")
print()
# print(pathNumTimestamps)
# print(pathClearances)   
# print(pathAvgGoalDistances)    

successCtr=0
minNumTimestamps=[INF]*len(MAPS)
maxNumTimestamps=[0]*len(MAPS)
sumNumTimestamps=[0]*len(MAPS)
avgNumTimestamps=[0]*len(MAPS)
minPathClearance=[INF]*len(MAPS)
maxPathClearance=[0]*len(MAPS)
sumPathClearance=[0]*len(MAPS)
avgPathClearance=[0]*len(MAPS)
minPathAvgGoalDistances=[INF]*len(MAPS)
maxPathAvgGoalDistances=[0]*len(MAPS)
sumPathAvgGoalDistances=[0]*len(MAPS)
avgPathAvgGoalDistances=[0]*len(MAPS)

for i in range(len(pathNumTimestamps[0])):
    allMapsPassed=True
    for j in range(len(MAPS)):
        if pathNumTimestamps[j][i]==INF:
            allMapsPassed=False
            break
    if allMapsPassed:
        successCtr+=1
    else:
        continue
    for j in range(len(MAPS)):
        minNumTimestamps[j]=min(minNumTimestamps[j],pathNumTimestamps[j][i])
        maxNumTimestamps[j]=max(maxNumTimestamps[j],pathNumTimestamps[j][i])
        sumNumTimestamps[j]=sumNumTimestamps[j]+pathNumTimestamps[j][i]
        minPathClearance[j]=min(minPathClearance[j],pathClearances[j][i])
        maxPathClearance[j]=max(maxPathClearance[j],pathClearances[j][i])
        sumPathClearance[j]=sumPathClearance[j]+pathClearances[j][i]
        minPathAvgGoalDistances[j]=min(minPathAvgGoalDistances[j],pathAvgGoalDistances[j][i])
        maxPathAvgGoalDistances[j]=max(maxPathAvgGoalDistances[j],pathAvgGoalDistances[j][i])
        sumPathAvgGoalDistances[j]=sumPathAvgGoalDistances[j]+pathAvgGoalDistances[j][i]
for j in range(len(MAPS)):    
    avgNumTimestamps[j]=sumNumTimestamps[j]/successCtr
    avgPathClearance[j]=sumPathClearance[j]/successCtr
    avgPathAvgGoalDistances[j]=sumPathAvgGoalDistances[j]/successCtr

print(minNumTimestamps)
print(maxNumTimestamps)
print(sumNumTimestamps)
print(avgNumTimestamps)
print()
print(minPathClearance)
print(maxPathClearance)
print(avgPathClearance)
print()
print(minPathAvgGoalDistances)
print(maxPathAvgGoalDistances)
print(avgPathAvgGoalDistances)
print()
print(successCtr)

print("Execution Time:",(time.time()-start_time)/60,"mins")
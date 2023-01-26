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

MAPS=[]
SUBGOALS=[]

# MAPS.append("small_map")
# SUBGOALS.append(AGENT_SUBGOALS2)

# MAPS.append("small_map2")
# SUBGOALS.append(AGENT_SUBGOALS3)

# MAPS.append("map2")
# SUBGOALS.append(AGENT_SUBGOALS4)

# MAPS.append("map3")
# SUBGOALS.append(AGENT_SUBGOALS5)

# MAPS.append("map4")
# SUBGOALS.append(AGENT_SUBGOALS6)

MAPS.append("squares_map")
SUBGOALS.append(AGENT_SUBGOALS7)

APF_DATA_ITER=150
APF_DATA_NO_ROTATE_KEEP=0.4
USE_CHECKPOINT=True
GOAL_DISTANCE_THRESHOLD=6
FILE_NUM=7

pathAPF=[]
pathBc=[]
pathDagger=[]

obstacles=[]
mapBackgrounds=[]
for map in MAPS:
    obstacles.append(initMap(mapObstaclesFilename=f"Maps/{map}_obstacles.txt"))
    mapBackgrounds.append(getMapBackground(mapImageFilename=f"Maps/{map}.png"))
pygame.init()

policyDagger=Policy()
daggerFile=f"WorkingCheckpoints/iter_checkpoint_{FILE_NUM}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}_8.pth"
policyDagger.loadModel(daggerFile)

policyBc=Policy()
bcFile=f"Checkpoints/checkpoint_{FILE_NUM}_{APF_DATA_ITER}_{APF_DATA_NO_ROTATE_KEEP}.pth"
policyBc.loadModel(bcFile)

keepIterating=True
for i in range(3): 
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

            if i==0:
                action=env.agentStates[0].selectAction("APF",apfParams=APF_PARAMS_S)
            elif i==1:
                action=env.agentStates[0].selectAction("NN",policyBc)
            elif i==2:
                action=env.agentStates[0].selectAction("NN",policyDagger)
            env.executeAction(action,noise=0.1,goalDistanceThreshold=GOAL_DISTANCE_THRESHOLD)
            env.render(screen,robotColor)
            pygame.display.update()

            robotPos=(int(env.agentPoses[0][0]),int(env.agentPoses[0][1]))
            if i==0:
                pathAPF.append(robotPos)
            elif i==1:
                pathBc.append(robotPos)
            elif i==2:
                pathDagger.append(robotPos)

            numTimestamps+=1
            goalDistance=euclidean((env.agentPoses[0][0],env.agentPoses[0][1]),(env.agentGoals[0][0],env.agentGoals[0][1]))

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
            
            if(numTimestamps>600):
                print("Time Limit Exceeded")
                collisionFlag=True
                break  

        if not keepIterating:
            break

        if not collisionFlag:
            print("Reached Goal!")
        else:
            allMapsPassed=False
            print("Collided!!!")
    
    if not keepIterating:
        break

env=Environment()
env.reset(obstacles=obstacles[0],agentRadius=AGENT_RADIUS,agentSubGoals=SUBGOALS[0])

pygame.display.set_caption(f"Scenario: easy")
screen=pygame.display.set_mode((mapBackgrounds[j].image.get_width(),mapBackgrounds[j].image.get_height()))
screen.blit(mapBackgrounds[0].image, mapBackgrounds[0].rect)
env.renderSubGoals(screen)

pathColors=[Colors.cyan,Colors.navy,Colors.magenta]
paths=[pathAPF,pathBc,pathDagger]
for i in range(3):
    for point in paths[i]:
        pygame.draw.circle(screen,pathColors[i],point,1)

pygame.display.update()
time.sleep(20)

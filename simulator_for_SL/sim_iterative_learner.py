from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
from config import *
import csv

pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")
obstacles=initMap(mapObstaclesFilename="small_map_obstacles.txt")
mapBackground=getMapBackground(mapImageFilename="small_map.png")

apfDataFilename = "apf_data.csv"
tempDataFilename = "iter_data.csv"

fields=["output_linear_velocity","output_angular_velocity","distance_from_goal","angle_from_goal"]
fields+=[f"lidar_depth_{i}" for i in range(1,1+NUMBER_OF_LIDAR_ANGLES)]

NUM_ITERATIONS=10
for i in range(NUM_ITERATIONS):
    
    print(f"\n***Iteration {i}***")
    env=Environment()
    env.reset(obstacles=obstacles,agentRadius=AGENT_RADIUS,agentSubGoals=AGENT_SUBGOALS)
    screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))
    print("Initialized map.")

    filename=tempDataFilename
    if(i==0):
        filename=apfDataFilename
    rows=[]

    running=True
    collisionFlag=False
    lastProgress=0
    while running:
        screen.blit(mapBackground.image, mapBackground.rect)
        for events in pygame.event.get():
            if events.type==pygame.QUIT:
                running=False
                pygame.quit()
                break
        
        action=env.agentStates[0].selectAction("NN",filename)
        # print(env.agentPoses[0],action)
        apfAction=env.agentStates[0].selectAction("APF")
        row=[apfAction[0],apfAction[1],
            env.agentStates[0].distanceGoal,
            env.agentStates[0].thetaGoal,
            ]+env.agentStates[0].lidarData[1]
        reward=env.executeAction(action,noise=0.1)
        env.render(screen)
        pygame.display.update()

        if(env.getAgentClearances()[0]==-1):
            print(env.getAgentClearances())
            inp=input()
            print("Robot Collided!!!")
            collisionFlag=True
            break

        if(env.agentProgress[0]>lastProgress):
            lastProgress+=1
            print(f"Robot progressed to Sub Goal {lastProgress} / {len(env.agentSubGoals[0])-1}.")

        if(abs(row[1])<abs(radians(1))):
            epsilon=random.uniform(0,1)
            if(epsilon<=0.1):
                rows.append(row)

        if euclidean((env.agentPoses[0][0],env.agentPoses[0][1]),(env.agentGoals[0][0],env.agentGoals[0][1]))<2:
            if (env.agentProgress[0]+1)==(len(env.agentSubGoals[0])-1):
                running=False
                break
    
    if not collisionFlag:
        print("Robot reached final goal!")

    print(f"Adding {len(rows)} rows to database.")
    with open(tempDataFilename,'w') as csvfile: 
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(fields)
        csvwriter.writerows(rows)
    with open(apfDataFilename,'a') as csvfile: 
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(rows)
        
        
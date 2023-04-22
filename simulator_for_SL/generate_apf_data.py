from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
from config import *
import csv

MAP_NAME="map3"
NUM_ITERATIONS=300
NO_ROTATE_KEEP=0.4
APF_DATA_NOISE=0.4

filename = f"Datasets/apf_data_{MAP_NAME}_{NUM_ITERATIONS}_{NO_ROTATE_KEEP}.csv"
obstacles=initMap(mapObstaclesFilename=f"Maps/{MAP_NAME}_obstacles.txt")
mapBackground=getMapBackground(mapImageFilename=f"Maps/{MAP_NAME}.png")

fields=["output_linear_velocity","output_angular_velocity","distance_from_goal","angle_from_goal"]
fields+=[f"lidar_depth_{i}" for i in range(1,1+NUMBER_OF_LIDAR_ANGLES)]
with open(filename,'w') as csvfile: 
    csvwriter = csv.writer(csvfile) 
    csvwriter.writerow(fields)
rows=[] 

for i in range(1,1+NUM_ITERATIONS):
    
    print(f"\n***Iteration {i}***")
    env=Environment()
    env.reset(obstacles=obstacles,agentRadius=AGENT_RADIUS,agentSubGoals=AGENT_SUBGOALS5)
    print("Initialized map.")

    running=True
    collisionFlag=False
    lastProgress=0
    while running:
        action=env.agentStates[0].selectAction()
        row=[action[0],action[1],
            env.agentStates[0].distanceGoal,
            env.agentStates[0].thetaGoal,
            ]+env.agentStates[0].lidarData[1]
        reward=env.executeAction(action,noise=APF_DATA_NOISE)

        if(env.getAgentClearances()[0]==-1):
            print(env.getAgentClearances())
            print("Robot Collided!!!")
            inp=input("Show Map? Enter Y/N: ")
            if(inp=="Y"):
                pygame.init()
                pygame.display.set_caption("Reactive Multi-Robot Navigation")
                screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))
                screen.blit(mapBackground.image, mapBackground.rect)
                env.render(screen)
                pygame.display.update()
                pygame.event.clear()
                while True:
                    event = pygame.event.wait()
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        break
            collisionFlag=True
            break

        if(env.agentProgress[0]>lastProgress):
            lastProgress+=1
            print(f"Robot progressed to Sub Goal {lastProgress} / {len(env.agentSubGoals[0])-1}.")

        if(abs(row[1])<abs(radians(1))):
            epsilon=random.uniform(0,1)
            if(epsilon<=NO_ROTATE_KEEP):
                rows.append(row)
        else:
            rows.append(row)

        if euclidean((env.agentPoses[0][0],env.agentPoses[0][1]),(env.agentGoals[0][0],env.agentGoals[0][1]))<2:
            if (env.agentProgress[0]+1)==(len(env.agentSubGoals[0])-1):
                print("Robot reached goal!")
                running=False
                break
    
    if not collisionFlag:
        print(f"Adding {len(rows)} rows to database.")
        with open(filename,'a') as csvfile: 
            csvwriter = csv.writer(csvfile)
            csvwriter.writerows(rows)
            rows=[]
        
        
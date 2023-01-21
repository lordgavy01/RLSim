from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
from config import *
import csv
import time

start_time=time.time()

MAPS=["small_map","small_map2","map2","map3","map4","squares_map"]
SUBGOALS=[AGENT_SUBGOALS2,AGENT_SUBGOALS3,AGENT_SUBGOALS4,AGENT_SUBGOALS5,AGENT_SUBGOALS6,AGENT_SUBGOALS7]
APF_PARAMS=[APF_PARAMS_1,APF_PARAMS_2,APF_PARAMS_1,APF_PARAMS_1,APF_PARAMS_3,APF_PARAMS_4]

NUM_ITERATIONS=150
NO_ROTATE_KEEP=0.4
APF_DATA_NOISE=0.4
GOAL_DISTANCE_THRESHOLD=6
FILE_NUM=7

obstacles=[]
mapBackgrounds=[]
for map in MAPS:
    obstacles.append(initMap(mapObstaclesFilename=f"Maps/{map}_obstacles.txt"))
    mapBackgrounds.append(getMapBackground(mapImageFilename=f"Maps/{map}.png"))

filename = f"Datasets/apf_data_{FILE_NUM}_{NUM_ITERATIONS}_{NO_ROTATE_KEEP}.csv"
fields=["output_linear_velocity","output_angular_velocity","distance_from_goal","angle_from_goal"]
fields+=[f"lidar_depth_{i}" for i in range(1,1+NUMBER_OF_LIDAR_ANGLES)]
# with open(filename,'w') as csvfile: 
#     csvwriter = csv.writer(csvfile) 
#     csvwriter.writerow(fields)
rows=[] 

for i in range(1,1+NUM_ITERATIONS):
    print(f"\n***Iteration {i}***")
    for j in range(len(MAPS)):
        print(f"\n*Map {j}*")
        env=Environment()
        env.reset(obstacles=obstacles[j],agentRadius=AGENT_RADIUS,agentSubGoals=SUBGOALS[j])

        running=True
        collisionFlag=False
        lastProgress=0
        ctr=0
        while running:
            ctr+=1
            action=env.agentStates[0].selectAction(apfParams=APF_PARAMS[j])
            row=[action[0],action[1],
                env.agentStates[0].distanceGoal,
                env.agentStates[0].thetaGoal,
                ]+env.agentStates[0].lidarData[1]
            reward=env.executeAction(action,noise=APF_DATA_NOISE)

            if(abs(row[1])<abs(radians(1))):
                epsilon=random.uniform(0,1)
                if(epsilon<=NO_ROTATE_KEEP):
                    rows.append(row)
            else:
                rows.append(row)

            if env.getAgentClearances()[0]==-1 or env.getAgentClearances()[0]<=3.5:
                for _ in range(4):
                    rows.append(row)

            if(env.getAgentClearances()[0]==-1):
                print(env.getAgentClearances())
                print("Robot Collided!!!")
                # inp=input("Show Map? Enter Y/N: ")
                # if(inp=="Y"):
                #     pygame.init()
                #     pygame.display.set_caption("Reactive Multi-Robot Navigation")
                #     screen=pygame.display.set_mode((mapBackgrounds[j].image.get_width(),mapBackgrounds[j].image.get_height()))
                #     screen.blit(mapBackgrounds[j].image, mapBackgrounds[j].rect)
                #     env.render(screen)
                #     pygame.display.update()
                #     pygame.event.clear()
                #     while True:
                #         event = pygame.event.wait()
                #         if event.type == pygame.QUIT:
                #             pygame.quit()
                #             break
                collisionFlag=True
                break

            if(env.agentProgress[0]>lastProgress):
                lastProgress+=1
                print(f"Robot reached Sub Goal {lastProgress} / {len(env.agentSubGoals[0])-1}.")

            if euclidean((env.agentPoses[0][0],env.agentPoses[0][1]),(env.agentGoals[0][0],env.agentGoals[0][1]))<GOAL_DISTANCE_THRESHOLD:
                if (env.agentProgress[0]+1)==(len(env.agentSubGoals[0])-1):
                    print("Robot reached goal!")
                    running=False
                    break
            
            if ctr>400:
                print("Time Limit Exceeded!!!")
                inp=input("Show Map? Enter Y/N: ")
                if(inp=="Y"):
                    pygame.init()
                    pygame.display.set_caption("Reactive Multi-Robot Navigation")
                    screen=pygame.display.set_mode((mapBackgrounds[j].image.get_width(),mapBackgrounds[j].image.get_height()))
                    screen.blit(mapBackgrounds[j].image, mapBackgrounds[j].rect)
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
        
        # if not collisionFlag:
        print(f"Adding {len(rows)} rows to database.")
        with open(filename,'a') as csvfile: 
            csvwriter = csv.writer(csvfile)
            csvwriter.writerows(rows)
            rows=[]

    print("Execution Time since start:",(time.time()-start_time),"s")
        
print("Execution Time:",(time.time()-start_time)/60,"mins")
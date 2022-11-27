from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
from config import *
import csv

pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")
mapBackground=getMapBackground(mapImageFilename="small_map.png")
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))
screen.blit(mapBackground.image, mapBackground.rect)
pygame.display.update()

obstacles=initMap(mapObstaclesFilename="small_map_obstacles.txt")
env=Environment()
env.reset(obstacles=obstacles,agentRadius=AGENT_RADIUS,agentSubGoals=AGENT_SUBGOALS)

running=True
key=0
print("Map Dimensions:",(mapBackground.image.get_width(),mapBackground.image.get_height()))

filename = APF_DATA_FILENAME
fields=["output_linear_velocity","output_angular_velocity","distance_from_goal","angle_from_goal"]
fields+=[f"lidar_depth_{i}" for i in range(1,1+50)]
rows=[]

while running:
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False
            pygame.quit()
            break
        if event.type==pygame.KEYDOWN:
            key_name=pygame.key.name(event.key)
            key_name=key_name.upper()
            if(key_name=="S"):
                with open(filename,'w') as csvfile: 
                    csvwriter = csv.writer(csvfile) 
                    csvwriter.writerow(fields) 
                    csvwriter.writerows(rows)
            if(key_name=="A"):
                with open(filename,'a') as csvfile: 
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerows(rows)
                    rows=[]
    
    user_input=pygame.key.get_pressed()
    if(user_input[pygame.K_UP] or user_input[pygame.K_w] or key%1==0):
        action=env.agentStates[0].selectAction()
        rows.append([action[0],action[1],
                    env.agentStates[0].distanceGoal,
                    env.agentStates[0].thetaGoal,
                    ]+env.agentStates[0].lidarData[1])
        reward=env.executeAction(action,NOISE)
        if euclidean((env.agentPoses[0][0],env.agentPoses[0][1]),(env.agentGoals[0][0],env.agentGoals[0][1]))<2:
            if (env.agentProgress[0]+1)==(len(env.agentSubGoals[0])-1):
                pygame.quit()
                break
        screen.blit(mapBackground.image, mapBackground.rect)
        env.render(screen)
        pygame.display.update()
    key+=1

inp=input()
if(inp=="S"):
    with open(filename,'w') as csvfile: 
        csvwriter = csv.writer(csvfile) 
        csvwriter.writerow(fields) 
        csvwriter.writerows(rows)
if(inp=="A"):
    with open(filename,'a') as csvfile: 
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(rows)
        rows=[]
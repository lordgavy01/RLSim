from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
import csv

pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))
screen.blit(mapBackground.image, mapBackground.rect)
pygame.display.update()

initMap()
env=Environment()
env.reset(obstacles=Polygons,agentRadius=10,agentSubGoals=[[(166, 99, 0), (225, 246, 0), (427, 249, 0), (607, 202, 0), (638, 65, 0)], [(635, 101, 0), (547, 228, 0), (285, 248, 0), (218, 244, 0), (199, 85, 0)], [(599, 516, 0), (425, 523, 0), (364, 451, 0), (334, 386, 0)], [(301, 340, 0), (345, 423, 0), (397, 500, 0), (482, 524, 0), (625, 493, 0)], [(451, 341, 0), (430, 420, 0), (393, 519, 0), (295, 557, 0)]])

running=True
key=0
print("Map Dimensions:",(mapBackground.image.get_width(),mapBackground.image.get_height()))

filename = "apf_data.csv"
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
        reward=env.executeAction(action,0.0)
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
from util import *
import csv

from lidar import *
from planner import *
from environment import *
from agent import *

initMap()
pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")

running=True
key=0
paused=False
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))

print("Map Dimensions:",(mapBackground.image.get_width(),mapBackground.image.get_height()))

env=Environment()
env.reset()

fields=["output_linear_velocity","output_angular_velocity","distance_from_goal","angle_from_goal"]
fields+=[f"lidar_depth_{i}" for i in range(1,1+50)]
rows=[]

while running:
    screen.blit(mapBackground.image, mapBackground.rect)
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
            break

    user_input=pygame.key.get_pressed()
    if(user_input[pygame.K_UP] or user_input[pygame.K_w]):
        action=env.agentStates[0].selectAction()
        rows.append([action[0],action[1],
                    env.agentStates[0].distanceGoal,
                    env.agentStates[0].thetaGoal,
                    ]+env.agentStates[0].lidarData[1])
        reward=env.executeAction(action,0.0)
    
    if euclidean((env.agentPoses[0][0],env.agentPoses[0][1]),(env.agentGoals[0][0],env.agentGoals[0][1]))<2:
        if (env.agentProgress[0]+1)==(len(env.agentSubGoals[0])-1):
            break

    env.render(screen)
    pygame.display.update()
    key+=1

filename = "APF_data.csv"
with open(filename,'w') as csvfile: 
    csvwriter = csv.writer(csvfile) 
    csvwriter.writerow(fields) 
    csvwriter.writerows(rows)
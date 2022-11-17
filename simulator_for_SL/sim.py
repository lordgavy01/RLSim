from util import *
from lidar import *
from planner import *
from environment import *
from agent import *

pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))
screen.blit(mapBackground.image, mapBackground.rect)
pygame.display.update()

initMap()
env=Environment()
env.reset(agentSubGoals=[[(166, 99, 0), (225, 246, 0), (427, 249, 0), (607, 202, 0), (638, 65, 0)], [(635, 101, 0), (547, 228, 0), (285, 248, 0), (218, 244, 0), (199, 85, 0)], [(599, 516, 0), (425, 523, 0), (364, 451, 0), (334, 386, 0)], [(301, 340, 0), (345, 423, 0), (397, 500, 0), (482, 524, 0), (625, 493, 0)], [(451, 341, 0), (430, 420, 0), (393, 519, 0), (295, 557, 0)]])

running=True
key=0
print("Map Dimensions:",(mapBackground.image.get_width(),mapBackground.image.get_height()))

while running:
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
            pygame.quit()
            break

    user_input=pygame.key.get_pressed()
    if(user_input[pygame.K_UP] or user_input[pygame.K_w] or key%20==0):
        action=env.agentStates[0].selectAction(policy="NN")
        reward=env.executeAction(action,0.0)
        if euclidean((env.agentPoses[0][0],env.agentPoses[0][1]),(env.agentGoals[0][0],env.agentGoals[0][1]))<2:
            if (env.agentProgress[0]+1)==(len(env.agentSubGoals[0])-1):
                pygame.quit()
                break
        screen.blit(mapBackground.image, mapBackground.rect)
        env.render(screen)
        pygame.display.update()
    
    key+=1

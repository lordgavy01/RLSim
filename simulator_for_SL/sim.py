from util import *

from lidar import *
from planner import *
from environment import *
from agent import *

initMap()
pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")

running=True
key=0
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))

print("Map Dimensions:",(mapBackground.image.get_width(),mapBackground.image.get_height()))

env=Environment()
env.reset(agentSubGoals=[[(134, 81, 0), (243, 262, 0), (421, 243, 0), (614, 92, 0)], [(624, 122, 0), (574, 226, 0), (326, 255, 0), (217, 259, 0), (205, 83, 0)], [(392, 68, 0), (409, 274, 0), (262, 311, 0), (101, 276, 0)], [(651, 460, 0), (560, 519, 0), (391, 518, 0), (350, 447, 0), (327, 369, 0)], [(444, 298, 0), (396, 450, 0), (270, 553, 0), (110, 541, 0), (74, 414, 0)]])

# env.agentStates[0].policyNN.train("APF_data.csv")

while running:
    screen.blit(mapBackground.image, mapBackground.rect)
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
            pygame.quit()
            break

    user_input=pygame.key.get_pressed()
    if(user_input[pygame.K_UP] or user_input[pygame.K_w]):
        action=env.agentStates[0].selectAction()
        reward=env.executeAction(action,0.2)
    
    env.render(screen)
    pygame.display.update()
    key+=1

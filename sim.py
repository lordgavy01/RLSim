# from util import *
from random import randint
from lidar import *
from planner import *

initMap()
pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")

    
addObject((275,390,pi/4),(689,236,0))

running=True
key=0
paused=False
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))

"""
Env_state{
    Robot's x,y,theta
    static_obstacles=list of polygons
    Dynamic Obstacles (humans) x,y,theta    
}
Robot_state{
    Lidar depths around robot
    relative direction of goal from robot
}

Action{
    velocity v
    angular w
    Delta time
}

def applyAction(env_oldState,action)  - Returns New env_State

def convert_env_state_to_robot_state(env_oldState)

def get_robot's_action(robot_state) - calls APF


"""
while running:
    screen.blit(mapBackground.image, mapBackground.rect)
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
            break
        if events.type==pygame.KEYDOWN:
            if events.key==pygame.K_SPACE:
                paused^=True
    user_input=pygame.key.get_pressed()
    X=positions[0][0]
    Y=positions[0][1]
    T=positions[0][2]
    if(user_input[pygame.K_UP] or user_input[pygame.K_w]):
        Y=Y-1
    elif(user_input[pygame.K_DOWN] or user_input[pygame.K_s]):
        Y=Y+1
    elif(user_input[pygame.K_LEFT] or user_input[pygame.K_a]):
        X=X-1
    elif(user_input[pygame.K_RIGHT] or user_input[pygame.K_d]):
        X=X+1
    if user_input[pygame.K_EQUALS]:
        T+=pi/64

    positions[0]=(X,Y,T)
    if paused:
        continue
    
    for object in positions:
        pygame.draw.circle(screen,(255,0,0),(object[0],object[1]),10)

    lidar_angles,lidar_depths,lidar_hitpoints=get_lidar_depths(positions[0],1e9,radians(90),60)
    for i in range(len(lidar_angles)):
        angle=lidar_angles[i]
        pygame.draw.line(screen,(0,255,0),(object[0],object[1]),lidar_hitpoints[i])    

    key+=1
    pygame.display.update()
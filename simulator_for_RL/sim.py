from util import *
from lidar import *
from planner import *
from environment import *
from agent import *
import torch
import torch.optim as optim
import logging

logging.basicConfig(filename="Train.log",format='%(asctime)s %(message)s', filemode='w')
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

initMap()
pygame.init()
pygame.display.set_caption("Reactive Multi-Robot Navigation")

running=True
key=0
paused=False
lr=1e-5
screen=pygame.display.set_mode((mapBackground.image.get_width(),mapBackground.image.get_height()))

print("Map Dimensions:",(mapBackground.image.get_width(),mapBackground.image.get_height()))
env=Environment()
env.reset()
optimizer=optim.Adam(env.agentStates[0].myPolicy.parameters(),lr)
bestPolicy=env.agentStates[0].myPolicy

while running:
    screen.blit(mapBackground.image, mapBackground.rect)
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
            break

    user_input=pygame.key.get_pressed()
    if(user_input[pygame.K_UP] or user_input[pygame.K_w]):
        scores=[]
        for i_episode in range(300):
            max_t=50
            gamma=0.95
            saved_log_probs = []
            rewards = []
            # env.reset()
            # env.agentStates[0].myPolicy=bestPolicy
            # Line 4 of pseudocode
            for t in range(max_t):
                lidarAngles,lidarDepths=env.agentStates[0].lidarData
                addInput=[]
                addInput.append(env.agentStates[0].distanceGoal)
                addInput.append(env.agentStates[0].thetaGoal)
                addInput+=lidarAngles
                addInput+=lidarDepths
                action, log_prob = env.agentStates[0].get_action_from_NN(np.array(addInput))
                saved_log_probs.append(log_prob)
                reward, done = env.executeAction(action)
                rewards.append(reward)
                if done:
                    break 
            scores.append(sum(rewards))
            discounts = [gamma**i for i in range(len(rewards)+1)]
            R = sum([a*b for a,b in zip(discounts, rewards)])
            policy_loss = []
            for log_prob in saved_log_probs:
                policy_loss.append(-log_prob * R)
            policy_loss = torch.cat(policy_loss).sum()
            if i_episode%100==0:
                logger.info("Loss at Episode "+str(i_episode)+" is "+str(policy_loss))
            optimizer.zero_grad()
            policy_loss.backward()
            optimizer.step()

        bestPolicy=env.agentStates[0].myPolicy
        print('Finished Training')
        # action=env.agentStates[0].selectAction()
        # reward=env.executeAction(action)
    elif user_input[pygame.K_DOWN]:
        lidarAngles,lidarDepths=env.agentStates[0].lidarData
        addInput=[]
        addInput.append(env.agentStates[0].distanceGoal)
        addInput.append(env.agentStates[0].thetaGoal)
        addInput+=lidarAngles
        addInput+=lidarDepths
        action, log_prob = env.agentStates[0].get_action_from_NN(np.array(addInput))
        env.executeAction(action)

    env.render(screen)
    pygame.display.update()
    key+=1


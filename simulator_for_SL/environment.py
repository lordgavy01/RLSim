from util import *
from agent import *
from lidar import *
from colors import *
from config import *

class Environment:

    # obstacles --> List of polygons. polygon --> [centre,vertices]
    # agentPoses, agentGoals --> List of (x,y,theta). theta is in radians 
    # agentStates --> List of objects of class AgentState.
    # agent 0 is robot, others are humans 
    
    def reset(self,obstacles,agentRadius=AGENT_RADIUS,agentSubGoals=AGENT_SUBGOALS):
        self.obstacles=obstacles
        self.agentStates=[]
        self.agentSubGoals=agentSubGoals
        self.agentRadius=agentRadius
        self.agentProgress=[0]*len(agentSubGoals)
        self.agentPoses=[]
        self.agentGoals=[]
        for agent in agentSubGoals:
            self.agentPoses.append(agent[0])
            self.agentGoals.append(agent[1])
        self.updateAgentStates()
    
    def render(self,screen,robotColor=(255,0,0)):
        agentColors=[Colors.red,Colors.blue,Colors.cyan,Colors.yellow,Colors.green]
        for i in range(len(self.agentPoses)):
            agentCoordinates=(int(self.agentPoses[i][0]),int(self.agentPoses[i][1]))
            goalCoordinates=(int(self.agentGoals[i][0]),int(self.agentGoals[i][1]))
            pygame.draw.circle(screen,agentColors[i],agentCoordinates,self.agentRadius)
            pygame.draw.circle(screen,agentColors[i],goalCoordinates,self.agentRadius,2)            
        rayColors=[Colors.green,Colors.blue]
        lidarAngles,lidarDepths=self.agentStates[0].lidarData
        for i in range(len(lidarAngles)):
            curAngle=normalAngle(self.agentPoses[0][2]+lidarAngles[i])
            robotCoordinates=(self.agentPoses[0][0],self.agentPoses[0][1])
            lidarHitpoint=(robotCoordinates[0]+(lidarDepths[i]+self.agentRadius)*cos(curAngle),
                           robotCoordinates[1]+(lidarDepths[i]+self.agentRadius)*sin(curAngle))
            if lidarDepths[i]>=1e9:
                pygame.draw.line(screen,rayColors[0],robotCoordinates,lidarHitpoint)
            else: 
                pygame.draw.line(screen,rayColors[1],robotCoordinates,lidarHitpoint)    

    def updateAgentStates(self,agentVelocities=[]):
        if(len(agentVelocities)==0):
            agentVelocities=[0 for i in range(len(self.agentSubGoals))]
        for i in range(len(self.agentPoses)):
            lidarData=get_lidar_depths(i,self.agentPoses,self.agentRadius,self.obstacles,max_lidar_distance=MAX_LIDAR_DISTANCE,
                                       field_of_view=FIELD_OF_VIEW,number_of_lidar_angles=NUMBER_OF_LIDAR_ANGLES)
            pose=self.agentPoses[i]
            goal=self.agentGoals[i]
            distanceGoal=euclidean((pose[0],pose[1]),(goal[0],goal[1]))
            thetaGoal=normalAngle(atan2(goal[1]-pose[1],goal[0]-pose[0])-pose[2])
            if not len(self.agentStates)==len(self.agentSubGoals):
                self.agentStates.append(AgentState(distanceGoal,thetaGoal,lidarData,agentVelocities[i]))
            else:
                self.agentStates[i].update(distanceGoal,thetaGoal,lidarData,agentVelocities[i])
    
    def executeAction(self,robotAction,noise=NOISE,goalDistanceThreshold=2):
        oldEnvironmentState=(self.obstacles,self.agentPoses,self.agentGoals,self.agentStates)
        agentVelocities=[]
        for i in range(len(self.agentPoses)):
            action=robotAction
            if not i==0:
                action=self.agentStates[i].selectAction()
            v=action[0]
            w=action[1]
            if i==0:
                rnoise=random.uniform(-noise,noise)
                v=min(max(v*(1+rnoise),0),VMAX)
                w=normalAngle(w*(1+rnoise))
            agentVelocities.append(v)
            self.agentPoses[i]=kinematic_equation(self.agentPoses[i],v,w,dT=1) 
            if euclidean((self.agentPoses[i][0],self.agentPoses[i][1]),(self.agentGoals[i][0],self.agentGoals[i][1]))<goalDistanceThreshold:
                if not (self.agentProgress[i]+1)==(len(self.agentSubGoals[i])-1):
                    self.agentProgress[i]+=1
                    self.agentGoals[i]=self.agentSubGoals[i][self.agentProgress[i]+1]
        self.updateAgentStates(agentVelocities)        
        newEnvironmentState=(self.obstacles,self.agentPoses,self.agentGoals,self.agentStates)
        return Environment.rewardFunction(oldEnvironmentState,robotAction,newEnvironmentState)
    
    def rewardFunction(oldEnvironmentState,robotAction,newEnvironmentState):
        return 0
    
    # NOTE: Would fail to detect collision if agent is completely inside obstacle
    def getAgentClearances(self):
        agentClearances=[]
        for agentId in range(len(self.agentSubGoals)):
            agentClearance=INF
            center=(self.agentPoses[agentId][0],self.agentPoses[agentId][1])
            radius=self.agentRadius
            for obstacle in self.obstacles:
                for i in range(len(obstacle)):
                    edge=(obstacle[i],obstacle[(i+1)%len(obstacle)])
                    d=getDistancePointLineSegment(center,edge)
                    if(d-radius<=0): 
                        agentClearance=-1
                    else: 
                        agentClearance=min(agentClearance,d-radius)
            for j in range(len(self.agentSubGoals)):
                if (agentId==j): continue
                center2=(self.agentPoses[j][0],self.agentPoses[j][1])
                radius2=self.agentRadius
                d=euclidean(center,center2)
                if(d-radius-radius2<=0): 
                    agentClearance=-1
                else: 
                    agentClearance=min(agentClearance,d-radius-radius2)
            agentClearances.append(agentClearance)
        return agentClearances
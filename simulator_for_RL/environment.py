from util import *
from agent import *
from lidar import *
from colors import *

class Environment:

    # obstacles --> List of polygons. polygon --> [centre,vertices]
    # agentPoses, agentGoals --> List of (x,y,theta). theta is in radians 
    # agentStates --> List of objects of class AgentState.
    # agent 0 is robot, others are humans 
    
    def reset(self,obstacles=Polygons,agentRadius=10,agentList=[((345,390,pi/4),(689,236,0)),
                                                                ((700,20,pi/2),(200,200,0)),
                                                                ((689,236,0),(275,390,0)),
                                                                ((355,360,0),(275,390,0)),]):
        self.obstacles=obstacles
        self.agentPoses=[]
        self.agentGoals=[]
        self.agentStates=[]
        for agent in agentList:
            self.addAgent(agent[0],agent[1])
        self.agentRadius=agentRadius
        self.updateAgentStates()
    
    def addAgent(self,agentPose,agentGoal):
        self.agentPoses.append(agentPose)
        self.agentGoals.append(agentGoal)

    def render(self,screen):
        agentColors=[Colors.red,Colors.blue,Colors.cyan,Colors.yellow,Colors.green]
        for i in range(len(self.agentPoses)):
            agentCoordinates=(self.agentPoses[i][0],self.agentPoses[i][1])
            goalCoordinates=(self.agentGoals[i][0],self.agentGoals[i][1])
            pygame.draw.circle(screen,agentColors[i],center=agentCoordinates,radius=self.agentRadius)
            pygame.draw.circle(screen,agentColors[i],center=goalCoordinates,radius=self.agentRadius,width=2)            
        rayColors=[Colors.green,Colors.blue]
        lidarAngles,lidarDepths=self.agentStates[0].lidarData
        for i in range(len(lidarAngles)):
            curAngle=normalAngle(self.agentPoses[0][2]+lidarAngles[i])
            robotCoordinates=(self.agentPoses[0][0],self.agentPoses[0][1])
            lidarHitpoint=(robotCoordinates[0]+lidarDepths[i]*cos(curAngle),
                           robotCoordinates[1]+lidarDepths[i]*sin(curAngle))
            if lidarDepths[i]>=1e9:
                pygame.draw.line(screen,rayColors[0],robotCoordinates,lidarHitpoint)
            else: 
                pygame.draw.line(screen,rayColors[1],robotCoordinates,lidarHitpoint)    

    def updateAgentStates(self):
        self.agentStates=[]
        for i in range(len(self.agentPoses)):
            lidarData=get_lidar_depths(i,self.agentPoses,self.agentRadius,self.obstacles,max_lidar_distance=1e9,
                                       field_of_view=radians(180),number_of_lidar_angles=50)
            pose=self.agentPoses[i]
            goal=self.agentGoals[i]
            distanceGoal=euclidean((pose[0],pose[1]),(goal[0],goal[1]))
            thetaGoal=normalAngle(atan2(goal[1]-pose[1],goal[0]-pose[0])-pose[2])
            self.agentStates.append(AgentState(distanceGoal,thetaGoal,lidarData))
    
    def executeAction(self,robotAction):
        oldEnvironmentState=(self.obstacles,self.agentPoses,self.agentGoals,self.agentStates)
        for i in range(len(self.agentPoses)):
            action=robotAction
            if not i==0:
                action=self.agentStates[i].selectAction()
            v=action[0]
            w=action[1]
            self.agentPoses[i]=kinematic_equation(self.agentPoses[i],v,w,dT=1)  
        self.updateAgentStates()        
        newEnvironmentState=(self.obstacles,self.agentPoses,self.agentGoals,self.agentStates)
        return self.rewardFunction(oldEnvironmentState,robotAction,newEnvironmentState)

    def checkCollision(self,newState):
        obstacles,newAgentPoses,_,_=newState
        for agents in newAgentPoses:
            for obs in obstacles:
                for pts in obs[1]:
                    if euclidean(agents,pts)<=3:
                        return True
        return False

    def rewardFunction(self,oldEnvironmentState,robotAction,newEnvironmentState):
        if self.checkCollision(newEnvironmentState):
            return (-200,0)
        _,_,_,oldAgentStates=oldEnvironmentState
        _,_,_,newAgentStates=newEnvironmentState
        r=oldAgentStates[0].distanceGoal-newAgentStates[0].distanceGoal
        if newAgentStates[0].distanceGoal<10:
            return (50,1)
        if r>=0:
            return (5**r,1)
        else:
            return (-5**r,1)
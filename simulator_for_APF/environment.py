from util import *
from agent import *
from lidar import *
from colors import *

class Environment:

    # obstacles --> List of polygons. polygon --> [centre,vertices]
    # agentPoses, agentGoals --> List of (x,y,theta). theta is in radians 
    # agentStates --> List of objects of class AgentState.
    # agent 0 is robot, others are humans 
    
    def reset(self,obstacles=Polygons,agentList=[((275,390,pi/4),(689,236,0)),
                                                 ((700,20,pi/2),(200,200,0))]):
        self.obstacles=obstacles
        self.agentPoses=[]
        self.agentGoals=[]
        self.agentStates=[]
        for agent in agentList:
            self.addAgent(agent[0],agent[1])
        self.updateAgentStates()
    
    def addAgent(self,agentPose,agentGoal):
        self.agentPoses.append(agentPose)
        self.agentGoals.append(agentGoal)

    def render(self,screen):
        agentColors=[Colors.red,Colors.blue]
        for i in range(len(self.agentPoses)):
            agentCoordinates=(self.agentPoses[i][0],self.agentPoses[i][1])
            goalCoordinates=(self.agentGoals[i][0],self.agentGoals[i][1])
            pygame.draw.circle(screen,agentColors[i],center=agentCoordinates,radius=10)
            pygame.draw.circle(screen,agentColors[i],center=goalCoordinates,radius=10,width=2)            
        rayColors=[Colors.blue,Colors.green]
        lidar_angles,lidar_depths,lidar_hitpoints=self.agentStates[0].lidarData
        for i in range(len(lidar_angles)):
            angle=lidar_angles[i]
            robotCoordinates=(self.agentPoses[0][0],self.agentPoses[0][1])
            if lidar_depths[i]>=1e9:
                pygame.draw.line(screen,rayColors[1],robotCoordinates,lidar_hitpoints[i])
            else: 
                pygame.draw.line(screen,rayColors[0],robotCoordinates,lidar_hitpoints[i])    

    def updateAgentStates(self):
        self.agentStates=[]
        for i in range(len(self.agentPoses)):
            lidarData=get_lidar_depths(self.obstacles,self.agentPoses[i],max_lidar_distance=1e9,field_of_view=radians(180),
                                       number_of_lidar_angles=50)
            pose=self.agentPoses[i]
            goal=self.agentGoals[i]
            distanceGoal=euclidean((pose[0],pose[1]),(goal[0],goal[1]))
            thetaGoal=normalAngle(goal[2]-pose[2])
            self.agentStates.append(AgentState(distanceGoal,thetaGoal,lidarData))
    
    def executeAction(self,robotAction):
        oldEnvironmentState=(self.obstacles,self.agentPoses,self.agentGoals,self.agentStates)
        for i in range(len(self.agentPoses)):
            action=robotAction
            if not i==0:
                action=self.agentStates[i].selectAction()
            self.agentPoses[i]=kinematic_equation(self.agentPoses[i],v=action[0],w=action[1],dT=1)  
        self.updateAgentStates()        
        newEnvironmentState=(self.obstacles,self.agentPoses,self.agentGoals,self.agentStates)
        return Environment.rewardFunction(oldEnvironmentState,robotAction,newEnvironmentState)
    
    def rewardFunction(oldEnvironmentState,robotAction,newEnvironmentState):
        return 0
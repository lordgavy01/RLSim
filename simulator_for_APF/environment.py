from util import *
from agent import *
from lidar import *
from colors import *

class Environment:

    # obstacles --> List of polygons. polygon --> [centre,vertices]
    # agentPoses, agentGoals --> List of (x,y,theta). theta is in radians 
    # agentStates --> List of objects of class AgentState.
    # agent 0 is robot, others are humans 
    
    def reset(self,obstacles=Polygons,agentRadius=10,agentSubGoals=[((91, 90,0),(201, 239),(543, 225,0),(619, 89,0)),
                                                                ((645, 223,0),(449, 227,0)),
                                                                ((666, 454,0),(71, 288,0)),
                                                                ((381, 95,0),(251, 249,0))]):
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
    
    # def addAgent(self,agentPose,agentGoal):
    #     self.agentPoses.append(agentPose)
    #     self.agentGoals.append(agentGoal)

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
            lidarHitpoint=(robotCoordinates[0]+(lidarDepths[i]+self.agentRadius)*cos(curAngle),
                           robotCoordinates[1]+(lidarDepths[i]+self.agentRadius)*sin(curAngle))
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
    
    def executeAction(self,robotAction,noise=0):
        oldEnvironmentState=(self.obstacles,self.agentPoses,self.agentGoals,self.agentStates)
        for i in range(len(self.agentPoses)):
            action=robotAction
            if not i==0:
                action=self.agentStates[i].selectAction()
            v=action[0]
            w=action[1]
            if i==0:
                r=random.uniform(-noise,noise)
                v=v*(1+noise)
                w=normalAngle(w*(1+noise))
            self.agentPoses[i]=kinematic_equation(self.agentPoses[i],v,w,dT=1) 
            if euclidean((self.agentPoses[i][0],self.agentPoses[i][1]),(self.agentGoals[i][0],self.agentGoals[i][1]))<2:
                if not (self.agentProgress[i]+1)==(len(self.agentSubGoals[i])-1):
                    self.agentProgress[i]+=1
                    self.agentGoals[i]=self.agentSubGoals[i][self.agentProgress[i]+1]
        self.updateAgentStates()        
        newEnvironmentState=(self.obstacles,self.agentPoses,self.agentGoals,self.agentStates)
        return Environment.rewardFunction(oldEnvironmentState,robotAction,newEnvironmentState)
    
    def rewardFunction(oldEnvironmentState,robotAction,newEnvironmentState):
        return 0
from util import *
from planner import *
from policyNN import *

class AgentState:

    # pose, goal --> x,y,theta(in radians)
    # lidarData --> 2 lists : lidarAngles(in radians), lidarDepths

    def __init__(self,distanceGoal,thetaGoal,lidarData,velocity):
        self.distanceGoal=distanceGoal        
        self.thetaGoal=thetaGoal
        self.lidarData=lidarData
        self.velocity=velocity
        self.policyNN=Policy()
    
    def update(self,distanceGoal,thetaGoal,lidarData,velocity):
        self.distanceGoal=distanceGoal        
        self.thetaGoal=thetaGoal
        self.lidarData=lidarData
        self.velocity=velocity

    # action --> (linearVelocity,angularVelocity)
    def selectAction(self,policy="APF"):
        if policy=="APF":
            bestAction=APF(self.distanceGoal,self.thetaGoal,self.lidarData,self.velocity)
        elif policy=="NN":
            lidarAngles,_=self.lidarData
            bestAction=self.policyNN.act(lidarAngles,[self.distanceGoal,self.thetaGoal])
            bestAction=bestAction[0].tolist()
        return bestAction
from util import *
from planner import *
from policyNN import *

class AgentState:

    # pose, goal --> x,y,theta(in radians)
    # lidarData --> 2 lists : lidarAngles(in radians), lidarDepths

    def __init__(self,distanceGoal,thetaGoal,lidarData):
        self.distanceGoal=distanceGoal        
        self.thetaGoal=thetaGoal
        self.lidarData=lidarData
        self.policyNN=Policy()

    # action --> (linearVelocity,angularVelocity)
    def selectAction(self):
        # bestAction=APF(self.distanceGoal,self.thetaGoal,self.lidarData)
        lidarAngles,_=self.lidarData
        bestAction=self.policyNN.act(lidarAngles,[self.distanceGoal,self.thetaGoal])
        return bestAction[0].tolist()
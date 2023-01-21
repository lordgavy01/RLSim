from util import *
from planner import *
from policy import *

class AgentState:

    # pose, goal --> x,y,theta(in radians)
    # lidarData --> 2 lists : lidarAngles(in radians), lidarDepths

    def __init__(self,distanceGoal,thetaGoal,lidarData,velocity):
        self.distanceGoal=distanceGoal        
        self.thetaGoal=thetaGoal
        self.lidarData=lidarData
        self.velocity=velocity
    
    def update(self,distanceGoal,thetaGoal,lidarData,velocity):
        self.distanceGoal=distanceGoal        
        self.thetaGoal=thetaGoal
        self.lidarData=lidarData
        self.velocity=velocity

    # action --> (linearVelocity,angularVelocity)
    def selectAction(self,algorithm="APF",policy=None,apfParams=APF_PARAMS_1):
        if algorithm=="APF":
            bestAction=APF(self.distanceGoal,self.thetaGoal,self.lidarData,self.velocity,apfParams)
        elif algorithm=="NN":
            _,lidarDepths=self.lidarData
            bestAction=policy.act(lidarDepths,[self.distanceGoal,self.thetaGoal])
            bestAction=bestAction[0].tolist()
        return bestAction
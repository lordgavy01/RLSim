from util import *
from planner import *

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
    def selectAction(self):
        bestAction=APF(self.distanceGoal,self.thetaGoal,self.lidarData,self.velocity)
        return bestAction
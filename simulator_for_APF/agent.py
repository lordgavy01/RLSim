from util import *
from planner import *

class AgentState:

    # pose, goal --> x,y,theta(in radians)
    # lidarData --> 3 lists : lidarAngles(in radians), lidarDepths, lidarHitpoints

    def __init__(self,distanceGoal,thetaGoal,lidarData):
        self.distanceGoal=distanceGoal        
        self.thetaGoal=thetaGoal
        self.lidarData=lidarData

    # action --> (linearVelocity,angularVelocity)
    def selectAction(self):
        bestAction=APF(self.distanceGoal,self.thetaGoal,self.lidarData)
        return bestAction
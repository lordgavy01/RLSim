from util import *
from planner import *

class AgentState:

    # pose, goal --> x,y,theta(in radians)
    # lidarData --> 2 lists : lidarAngles(in radians), lidarDepths

    def __init__(self,distanceGoal,thetaGoal,lidarData):
        self.distanceGoal=distanceGoal        
        self.thetaGoal=thetaGoal
        self.lidarData=lidarData

    # action --> (linearVelocity,angularVelocity)
    def selectAction(self):
        bestAction=APF(self.distanceGoal,self.thetaGoal,self.lidarData)
        return bestAction
    
    # def get_action_from_NN(self):

    # # episodes_data --> List of tuples. Each tuple --> (s,a,log_prob,r,s_dash)
    # def update_NN(self,episodes_data):
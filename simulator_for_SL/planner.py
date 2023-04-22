from util import *
from config import *

APF_PARAMS=APF_PARAMS1

def APF(distanceGoal,thetaGoal,lidarData,oldV,vmax=VMAX,wmax=WMAX):
    lidarAngles,lidarDepths=lidarData
    
    # Attraction Modelling
    kAttr=APF_PARAMS.kAttr
    distanceThresholdAttraction=APF_PARAMS.distanceThresholdAttraction

    if distanceGoal<=distanceThresholdAttraction:
        fAttr=(kAttr*(distanceGoal),thetaGoal)
    else:
        fAttr=(distanceThresholdAttraction*kAttr,thetaGoal)

    # Repulsion Modelling
    kRep=APF_PARAMS.kRep
    sigma=APF_PARAMS.sigma

    fRep=(0,0)
    for i in range(len(lidarAngles)):
        obsAngle=lidarAngles[i]
        obsDistance=lidarDepths[i]
        curFMagnitude=kRep*exp(-obsDistance/sigma)
        curFTheta=normalAngle(-obsAngle)
        fRep=addForces(fRep,(curFMagnitude,curFTheta))

    # Converting resultant force to (linear velocity,angular velocity) for non-holonomic robot
    fRes=addForces(fAttr,fRep)
    kParam=APF_PARAMS.kParam
    w=normalAngle(kParam*(fRes[1]))
    if abs(w)>wmax:
        w=wmax*(w/abs(w))
    v=min(max(oldV+kParam*fRes[0]*cos(fRes[1]),0),vmax)

    bestAction=(v,w)
    return bestAction
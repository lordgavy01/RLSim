from util import *

def APF(distanceGoal,thetaGoal,lidarData,oldV,vmax=2,wmax=radians(20)):
    lidarAngles,lidarDepths=lidarData
    
    # Attraction Modelling
    kAttr=50
    distanceThresholdAttraction=1

    if distanceGoal<=distanceThresholdAttraction:
        fAttr=(kAttr*(distanceGoal),thetaGoal)
    else:
        fAttr=(distanceThresholdAttraction*kAttr,thetaGoal)

    # Repulsion Modelling
    kRep=1e5
    distanceThresholdRepulsion=400

    fRep=(0,0)
    for i in range(len(lidarAngles)):
        obsAngle=lidarAngles[i]
        obsDistance=lidarDepths[i]
        if obsDistance>=distanceThresholdRepulsion:
            continue
        curFMagnitude=kRep*(1/obsDistance-1/distanceThresholdRepulsion)*(1/(obsDistance**2))
        curFTheta=normalAngle(-obsAngle)
        fRep=addForces(fRep,(curFMagnitude,curFTheta))

    # Converting resultant force to (linear velocity,angular velocity) for non-holonomic robot
    fRes=addForces(fAttr,fRep)
    kParam=0.5
    w=normalAngle(kParam*(fRes[1]))
    if abs(w)>wmax:
        w=wmax*(w/abs(w))
    v=min(kParam*fRes[0],vmax)
    # v=min(max(oldV+kParam*fRes[0]*cos(fRes[1]),0),vmax)

    bestAction=(v,w)
    return bestAction
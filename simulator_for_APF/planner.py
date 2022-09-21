from util import *

def APF(distanceGoal,thetaGoal,lidarData,vmax=10,wmax=radians(180)):
    lidarAngles,lidarDepths=lidarData
    
    # Attraction Modelling
    kAttr=10
    distanceThresholdAttraction=1

    if distanceGoal<=distanceThresholdAttraction:
        fAttr=(kAttr*(distanceGoal),thetaGoal)
    else:
        fAttr=(distanceThresholdAttraction*kAttr,thetaGoal)

    # Repulsion Modelling
    kRep=0
    distanceThresholdRepulsion=20

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
    v=min(kParam*fRes[0],vmax)

    bestAction=(v,w)
    return bestAction
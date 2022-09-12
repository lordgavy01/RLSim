def findPath(source,goal,algo=None):
    if algo=='apf':
        return APF(source,goal)
    

def APF(robotPose,goal,lidarData):
    robot_x,robot_y,robot_theta=robotPose
    goal_x,goal_y,goal_theta=goal
    lidar_angles,lidar_depths=lidarData
    bestAction=None







    return bestAction
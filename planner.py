def findPath(source,goal,algo=None):
    if algo=='apf':
        return APF(source,goal)
    

def APF(source,goal):
    return (source,goal)
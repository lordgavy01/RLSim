from util import*

def get_lidar_depths(obstacles,robotPose,max_lidar_distance,field_of_view=radians(180),number_of_lidar_angles=50):
  # Assuming robot_theta (with positive x-axis) and field_of_veiew are in radians
  robot_x,robot_y,robot_theta=robotPose
  lidar_angles=[]  
  angle_spacing=(field_of_view)/(number_of_lidar_angles-1)
  for i in range(number_of_lidar_angles-1):
    theta=field_of_view/2-i*angle_spacing
    lidar_angles.append(normalAngle(theta))
  lidar_angles.append(normalAngle(-field_of_view/2))
  if number_of_lidar_angles==1 :
    lidar_angles=[0]

  lidar_depths=[]
  for lidar_angle in lidar_angles:
    cur_angle=normalAngle(lidar_angle+robot_theta)
    min_distance=INF
    checker_line=((robot_x,robot_y),(robot_x+INF*cos(cur_angle),robot_y+INF*sin(cur_angle)))
    for centre,obstacle in obstacles:
      for i in range(len(obstacle)):
        edge=(obstacle[i],obstacle[(i+1)%len(obstacle)])        
        p=getIntersection(edge,checker_line)
        if(p==False):
          continue
        if(euclidean((robot_x,robot_y),p)<min_distance):
          min_distance=euclidean((robot_x,robot_y),p)
    if min_distance>=max_lidar_distance:
      min_distance=max_lidar_distance
    lidar_depths.append(min_distance)
  return lidar_angles,lidar_depths

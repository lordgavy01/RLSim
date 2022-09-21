from util import*
INF=1e18

def get_lidar_depths(obstacles,robotPose,max_lidar_distance,field_of_view=radians(90),number_of_lidar_angles=20):
  # Assuming robot_theta (with positive x-axis) and field_of_veiew are in radians
  robot_x,robot_y,robot_theta=robotPose
  lidar_angles=[]  
  left_theta=robot_theta+field_of_view/2
  right_theta=robot_theta-field_of_view/2
  angle_spacing=(field_of_view)/(number_of_lidar_angles-1)

  for i in range(number_of_lidar_angles-1):
    theta=left_theta-i*angle_spacing
    lidar_angles.append(normalAngle(theta))
  lidar_angles.append(normalAngle(right_theta))

  if number_of_lidar_angles==1 :
    lidar_angles=[robot_theta]

  lidar_depths=[]
  lidar_hitpoints=[]
  for lidar_angle in lidar_angles:
    min_distance=INF
    hitpoint=(INF,INF)
    checker_line=((robot_x,robot_y),(robot_x+INF*cos(lidar_angle),robot_y+INF*sin(lidar_angle)))
    for centre,obstacle in obstacles:
      for i in range(len(obstacle)):
        edge=(obstacle[i],obstacle[(i+1)%len(obstacle)])        
        p=getIntersection(edge,checker_line)
        if(p==False):
          continue
        if(euclidean((robot_x,robot_y),p)<min_distance):
          min_distance=euclidean((robot_x,robot_y),p)
          hitpoint=p
    if min_distance==INF or min_distance>max_lidar_distance:
      min_distance=max_lidar_distance
      hitpoint=(robot_x+max_lidar_distance*cos(lidar_angle),robot_y+max_lidar_distance*sin(lidar_angle))
    lidar_depths.append(min_distance)
    lidar_hitpoints.append(hitpoint)
  return lidar_angles,lidar_depths,lidar_hitpoints

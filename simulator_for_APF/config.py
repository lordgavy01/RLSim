from math import *

MAP_IMAGE_FILENAME="map.png"
MAP_OBSTACLES_FILENAME="map_obstacles.txt"

NOISE=0.2

AGENT_RADIUS=10
AGENT_SUBGOALS=[((91, 90,0),(201, 239),(543, 225,0),(619, 89,0)),
                ((645, 223,0),(449, 227,0)),
                ((666, 454,0),(71, 288,0)),
                ((381, 95,0),(251, 249,0))]

FIELD_OF_VIEW=radians(180)
NUMBER_OF_LIDAR_ANGLES=50
MAX_LIDAR_DISTANCE=1e9

VMAX=2
WMAX=radians(20)
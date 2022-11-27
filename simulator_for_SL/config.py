from math import *

MAP_IMAGE_FILENAME="map.png"
MAP_OBSTACLES_FILENAME="map_obstacles.txt"
APF_DATA_FILENAME="apf_data.csv"

NOISE=0.0

AGENT_RADIUS=10
AGENT_SUBGOALS=[[(166, 99, 0), (225, 246, 0), (427, 249, 0), (607, 202, 0), (638, 65, 0)], [(635, 101, 0), (547, 228, 0), (285, 248, 0), (218, 244, 0), (199, 85, 0)], [(599, 516, 0), (425, 523, 0), (364, 451, 0), (334, 386, 0)], [(301, 340, 0), (345, 423, 0), (397, 500, 0), (482, 524, 0), (625, 493, 0)], [(451, 341, 0), (430, 420, 0), (393, 519, 0), (295, 557, 0)]]

FIELD_OF_VIEW=radians(180)
NUMBER_OF_LIDAR_ANGLES=50
MAX_LIDAR_DISTANCE=1e9

VMAX=2
WMAX=radians(20)
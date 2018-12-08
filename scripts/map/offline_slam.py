"""
This file is given to the students for the SLAM project. It reads map data from map_data.txt and plays it through
slam_helper.py

map_data format:
all on a single line: a list of length 3 lists where each inner list holds the kp, des, then ir reading
"""

# most amazing and greatest python library
import ast
from slam_helper import FastSLAM
import numpy as np
import string
import os

flight_path = 'flight_data.txt'

file = open(flight_path, 'r')

map_data = ""
for i, char in enumerate(file.read()):
    if ord(char) > 32 and ord(char) <= 126: # don't write spaces, non alphanumeric things
        map_data += char

# get rid of weird numpy words
map_data = string.replace(map_data, ",dtype=uint8)", "")
map_data = string.replace(map_data, "array(", "")

unprocessed_map_data = ast.literal_eval(map_data)

# data[0] is kp, data[1] is des, and data[2] is range reading
map_data = []
for i, data in enumerate(unprocessed_map_data):
    if (data[1] == None): # avoid using camera frames when drone was on the ground
        continue
    des_list = np.asarray(data[1], dtype=np.uint8)
    """
    for d in data[1]:
        for x in d:
            x = x.astype(np.uint8)
    """
    map_data.append((data[0], des_list, data[2]))

slam_estimator = FastSLAM()
slam_estimator.generate_particles(40)

print "Starting SLAM"
for i in range(1, len(map_data)):
    slam_estimator.run(map_data[i-1][2], map_data[i-1][0], map_data[i-1][1], map_data[i][0], map_data[i][1])

particle = slam_estimator.particles[0]

# so we want to extract the lists of landmark kp and des from this particle
landmark_keypoints = [[ln.x, ln.y] for ln in particle.landmarks]
landmark_descriptors = [ln.des for ln in particle.landmarks]

# in the future, could do something like filter the landmarks by their count...

print "Writing map to file"
map_file = open("map.txt", 'w')
map_file.write(str(landmark_keypoints))
map_file.write('\n')
map_file.write(str(landmark_descriptors))
map_file.close()
print "Finished"

"""
This file is given to the students for the SLAM project. It reads map data from map_data.txt and plays it through
slam_helper.py

map_data format:
all on a single line: a list of length 3 lists where each inner list holds the kp, des, then ir reading
"""

# so the ast function claims there's an EOF in there, but the readline thing reads
# until EOF so that's basically impossible, I suspect it just has a length limit

# most amazing and greatest python library
import ast
from slam_helper import FastSLAM
import numpy as np
import string

map_path = 'flight_data.txt'

file = open(map_path, 'r')

#map_data = file.readline()
#file.close()

map_data = ""
for char in file.read():
    if ord(char) > 32 and ord(char) <= 126: 
        map_data += char

# get rid of weird numpy wrods
map_data = string.replace(map_data, ",dtype=uint8)", "")
map_data = string.replace(map_data, "array(", "")


unprocessed_map_data = ast.literal_eval(map_data)

map_data = []
for data in unprocessed_map_data:
    des_list = np.asarray(data[1], dtype=np.uint8)
    map_data.append((data[0], des_list, data[2]))

slam_estimator = FastSLAM()
slam_estimator.generate_particles(15)

for i in range(1, len(map_data)):
    print slam_estimator.run(map_data[i-1][2], map_data[i-1][0], map_data[i-1][1], map_data[i][0], map_data[i][1])









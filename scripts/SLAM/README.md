# PiDrone SLAM Instructions

Change NUM_PARTICLES and NUM_FEATURES in slam/offboard_slam.py to increase the accuracy of SLAM, but decrease the speed.

## Run SLAM off-board

You will need to have utils.py, slam_helper.py, and offboard_slam.py on the offboard computer. Run offboard_slam.py in window 4 in screen. Press 'r' to start SLAM, and you will see poses printed to the terminal. On the pi, you will need to have image_pub.py running in window 4 in order for the offboard computer to recieve the image stream.

## Run SLAM on-board

You will need to have utils.py, slam_helper.py, and slam.py on the pi. Run slam.py in window 4 in screen. Press 'r' to start SLAM, and you will see poses printed to the terminal.



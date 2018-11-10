To run SLAM offboard/offline:

Run "vision_localization_offboard.py --offline" as the vision node. You can press 'r' in the command line interface or 
on the web interface to start and stop collecting flight data. When you stop collecting it, the data will be written to a file
called "flight_data.txt."

To actually run SLAM, you can use the file "offboard_offline.py" in the map folder. This will read data from
"flight_data.txt" and use the SLAM implementation located in "slam_helper.py" to run SLAM. The poses of the particles
(with landmarks) from SLAM with then be written to "pose_data.txt" which "animate_slam.py" looks for.

To visualize the flight along with the map, run "animate_slam.py", this requires MatPlotLib.
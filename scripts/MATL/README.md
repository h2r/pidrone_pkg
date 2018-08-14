


# MATL
"MATL" stands for *Mapping and then Localization.*

The code here presents an alternative to running SLAM onboard in real time, which is to fly the drone while saving the camera data, then land and run SLAM on the ground, building a map. Finally, you fly the drone a second time running localization with the map you just made.

To use this code, use the index.html file for the web interface, and move the two MATL files into scripts. You run MATL.py, and press 'm' once to being mapping. Once you are done mapping the area over which you wish to localize, land and hit 'm' again. It will build the map (slowly) and tell you when it is done. Finally, hit 'r' to toggle localization and fly!

Note: it does not work very well.

# Localization

## Create a map
1. Use capture_images.py to take photo by pressing r (you can change CAMERA_WIDTH 
and CAMERA_HEIGHT), or use any other camera.
2. Take the images under 60cm, use 
AutoStitch or any other way to stitch images as a map.
3. It's better to keep the map with 2048 pixels corresponding to 1.4 meters so you
can keep most of the parameters.
4. MAP_PIXEL_WIDTH, MAP_PIXEL_HEIGHT, MAP_REAL_WIDTH, MAP_REAL_HEIGHT should be changed 
in both picam_localization_distance.py and global_position_estimator_distance.py

## Verify the map
1. Use capture_images.py to take several images, and use test python files to verify
that it can find matches on the map
2. nfeatures can be tuned, or make the map with more features (the right-top image of 
map.jpg don't works well with current setting).

## Fly
1. arm, takeoff, when it stable, press r to start localization, then, press p to do 
the position hold (run picam_localization_distance.py instead of flow_pub_transform.py).
2. control the drone by press i, j, k, l, each pressing will increase or decrease the target 
pose by 0.1 meters
3. If the estimated pose is not correct, change the CAMERA_SCALE.

## Attention
Before flying, you can run picam_localization_distance.py without state_controller.py, holding the 
drone and move around to see the output in terminal.
Due to battery or the code, it's not very stable.

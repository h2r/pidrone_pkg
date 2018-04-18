1. Use capture_images.py to take photo by pressing r (you can change CAMERA_WIDTH and CAMERA_HEIGHT), or use any other camera.
2. Record the image width and height, also the length of image in real world.
3. Replace the value of MAP_WIDTH, MAP_HEIGHT, MAP_REAL_WIDTH, MAP_REAL_HEIGHT according to your image in feature_test.py.
   To test the map has enough features, you need to take a photo on a small area of map with lower height. img1 is the current image, img2 is the map.
   If the map does not have enough features, you need to increase the nfeatures in ORB.
4. In flow_pub_position.py, you need to pass 4 maps (with same resolution), map1, map2, map3, map4, in the real world it looks like this
   map2, map3
   map1, mpa4
5. Takeoff, when the drone is stable, press r and then p. Use i,j,k,l to move the drone and you can see the current pose and target pose in window of running flow_pub_position.py.

nfeatures for ORB are free to adjust.
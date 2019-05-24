

# PiDrone Vision Instructions


Version 2.0 of the PiDrone software comes with 5 programs for vision outlined in the table below. The two main algorithms are Localization and SLAM (Simultaneous Localization and Mapping). They both use the drone’s camera to produce a pose estimate, but localization requires a map of the environment before hand (map.jpg) whereas SLAM builds a map of the environment as it runs. SLAM is much slower than localization. The table shows which of the algorithms are supported under the following conditions: onboard (running on the raspberry pi), offboard (running on a separate Linux computer), online (running in real time) and offline (collecting data first and then computing in retrospect).

|          | Online              | Offline       |
|----------|---------------------|---------------|
| Onboard  | Localization & SLAM | SLAM          |
| Offboard | Localization & SLAM | Not Supported |


As you can see, nothing is supported onboard and offline because the offboard computer is assumed to be quick enough to run the programs in real time. Also, localization is not supported onboard and offline because localization cannot run
in retrospect, only in real time. Finally, note that *SLAM onboard and online is supported, but runs too slow to work accurately.* Do not expect this program to produce accurate poses.

The following instructions describe how to use each of the five supported programs:


## Localization - Online

**Onboard:** run "vision_localization_onboard.py" on the pi. You must fly over the area captured in map.jpg. Press ‘r’ to toggle localization.

**Offboard:** run "vision_localization_offboard.py" on the pi and "offboard_localization.py" on the remote computer. You must fly over the area captured in map.jpg. Press ‘r’ to toggle localization.


**Change the map:** It is easiest to take photos of the new map with a cell phone or other camera. Take them at a height of 25cm and use an image stitching software to generate the map. We recommend auto-stitch. Replace map.jpg with your new map and change the following four parameters in "offboard_localization.py", "onboard_localization.py", and "localization_helper.py": MAP_PIXEL_WIDTH, MAP_PIXEL_HEIGHT, MAP_REAL_WIDTH, MAP_REAL_HEIGHT. You may need to resize the image to be smaller if it is too large.

## SLAM - online

**Offboard:** run "vision_localization_offboard.py" on the pi. On the offboard computer run "offboard_slam.py." Press ‘r’ to toggle SLAM.

**Onboard:** run "vision_localization_onboard.py --SLAM" on the pi. Press 'r' to toggle localization.


## SLAM - offline

**Onboard**: run "vision_localization_onboard.py --SLAM --offline" on the pi. Press “m” to toggle mapping mode on, during which time you can fly to collect data for the map. Pressing “m” again will stop the mapping and begin running SLAM offline. It is highly recommended that you land the drone for this part. Once it tells you that it is finished making the map, press “r” to toggle localization over the map you have just made.


## Mapping for Localization

### Create the map:
You may use Microsoft Powerpoint or any other software to fill the white paper with symbols, emojis, logos, and icons (black and white would be enough). Then, print them and stitch them together. The material of the paper should not like the poster which reflects much light (no shiny surface). We have map in CIT 121, which would be a good example of how the map looks like.

### Take photos of the map:
The height of the camera should be from 30-60cm (so you may take serveral images to cover the whole map). Ideally it would be 55cm. The camera should be level. You may use some apps to ensure that your phone is level while you taking the photo. Also, there should be some overlap (5-10cm) between images, so the software can group the images based on overlaps.

### Stitch images as a map:
I recommand the microsoft image composite for stitching. The final image should be compressed in terms of the ratio like 5500 pixels to 3 meters.

### Change parameters in code:
In `offboard_localization.py`, map parameters should be changed to fit the new map. The name of the map should be changed in the code `create_map('map.jpg')`.
In `localization_helper.py`, map parameters should be the same as above. The `ORB_GRID_SIZE_X` and `ORB_GRID_SIZE_Y` should be changed as well. The ratio between the grid size and resolution should be similar to previous setting.

### Tips:
It's a good idea to test the localization by holding the drone before flying it.



Happy Flying!


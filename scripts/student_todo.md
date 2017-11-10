# STUDENT TODO

## Create the picam_pos_class.py file

## Create the PosAnalyzer Class

* class AnalyzePos(picamera.array.PiMotionAnalysis)
* implements functions write(data) where data contains the newest frame
* implements setup(res) where res is a tuple of the dimensions of the image

## Get the pos from the image

* convert the image data np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
* run cv2.estimateRigidTransform and [extract the camera translation](http://nghiaho.com/?p)
* save the first image. when you are seeing the first image you know your absolute position
* integrate position by comparing frames to the previous frame when you dont see the first image
* be careful to blend you integrated position estimate back in with your first frame to avoid jumps

## set up the publishers and subscribers

* subscribe to /pidrone/toggle_transform and enable or disable your position hold
* subscribe to /pidrone/reset_transform to take a new first frame
* (optional) subscribe to /pidrone/infrared to get your current altitude
* subscribe to /pidrone/set_mode to get commands from the javascript interface
* publish commands to /pidrone/set_mode_vel to send commands to state_controllers velocity controller
* when position hold is not eabled, you should forward commands from set_mode to set_mode_vel

## write a controller

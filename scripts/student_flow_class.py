import numpy as np
import picamera
import picamera.array
import cv2
from h2rMultiWii import MultiWii
import time
from pidrone_pkg.msg import axes_err
  

class AnalyzeFlow(picamera.array.PiMotionAnalysis):

# Please fill in this function to get the average x and y velocity. Make sure
# that you output your final velocity estimates in the self.x_motion and
# self.y_motion variables. These are accessed from outside your program
    def analyse(self, a):
        x = a['x']
        y = a['y']
        self.x_motion = self.alpha * (-np.sum(x) * self.flow_scale) + (1. - self.alpha) * self.x_motion
        self.y_motion = self.alpha * (np.sum(y) * self.flow_scale) + (1. - self.alpha) * self.y_motion
        

# Please fill in this setup function with the parameters you need to initialize.
# Make sure that you initialize self.x_motion, self.y_motion, and any other
# parameters you may need. Note that flow_scale is a scale factor that you can
# divide your flow estimates by in order to normalize them. Note that you will
# also need to figure out the conversion factor (a scalar) that you need to
# multiply your normalized flow vectors by in order to convert the scale to
# centimeters
    def setup(self, camera_wh = (320,240), pub=None, flow_scale = 0.165):
        self.flow_scale = 0.00004
        self.x_motion = 0
        self.y_motion = 0
        self.old_smooth_x = 0
        self.old_smooth_y = 0
        self.alpha = 0.3

def flow_angle_comp(raw_flow_x, raw_flow_y, d_theta_x_dt, d_theta_y_dt):
    #print np.sign(raw_flow_x) ,np.sign(np.tan(d_theta_x_dt)) ,np.sign(raw_flow_y), np.sign(np.tan(d_theta_y_dt)) 
    
    flow_x = raw_flow_x - np.tan(d_theta_x_dt) * 0.008
    flow_y = raw_flow_y - np.tan(d_theta_y_dt) * 0.008
    return flow_x, flow_y

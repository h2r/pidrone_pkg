import numpy as np
import picamera
import picamera.array
import cv2
from h2rMultiWii import MultiWii
import time
from pidrone_pkg.msg import axes_err
  

class AnalyzeFlow(picamera.array.PiMotionAnalysis):

    def analyse(self, a):
        x = a['x']
        y = a['y']
        self.x_motion =  -np.sum(x) * self.flow_coeff # + np.arctan(self.ang_vx * diff_time) * self.ang_coefficient
        self.y_motion = np.sum(y) * self.flow_coeff # + np.arctan(self.ang_vy * diff_time) * self.ang_coefficient

    def setup(self, camera_wh = (320,240), pub=None, flow_scale = 16.5):
        self.x_motion = 0
        self.y_motion = 0
        self.max_flow = camera_wh[0] / 16.0 * camera_wh[1] / 16.0 * 2**7
        self.norm_flow_to_cm = flow_scale # the conversion from flow units to cm
        self.flow_coeff = self.norm_flow_to_cm/self.max_flow

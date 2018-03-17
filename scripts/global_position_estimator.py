import rospy
import math
import random
# first, refactor flow_pub_transform so that it computes the features
# explicitly, and then calls cv2.estimateRigidTransform using those
# features.  Verify position hold still works.  It should be faster. 
#

# second, compute the local pose for each feature.  Camera frame.  For
# each feature, you'll have an x,y,z in the camera frame (where the
# camera is zero, zero,zero).  Verify it's right by publishing the
# pose as a ROS marker array.  Visualize it in rviz.

# third, make a feature map.  So given the global true pose of the
# robot from the mocap (Optitrak), store the global x,y,z location of each
# feature by transforming above to the global frame using the mocap.

# use time synchronizer to line up optitrak poses with images.
# http://docs.ros.org/api/message_filters/html/python/#message_filters.TimeSynchronizer

# 

# fourth, write out the math and agree how it will work.

# publish the features in the camera frame in ros

# this node subscribes and does an update given the map and each feature.

# this node publishes a TF transform from /map to /base_link

# Use meters (not centimers) and radians.

class LocalizationParticleFilter:
    """
    Particle filter for localization.  
    """
    
    def __init__(self):
        self.map = []

        # parameters - noise on sensor.  
        self.sigmar = 0.1
        self.sigmaphi = 0.1
        self.particles = [] # initialize to random particles spread over map.
    
    def sample_motion_model(self, ut, xt):
        """
        Implement motion model from Equation 3 in PiDrone Slam with noise. 
        """
        # compute xtp1, with equation 3.  
        xtp1 = 0 # A * xt + B * ut

        # add random gaussian noise, but use matpl.
        # xtp1 += np.random.normal(na.zeros(xt.shape), self.sigma)
        return xtp1

    def measurement_model(self, zt, xt):
        """
        landmark_model_known_correspondence from probablistic robotics
        """
        x,y,z = xt
        j, r, phi = zt
        mx, my = self.map[j]
        hatr = math.sqrt(math.pow(mx - x,2) + math.pow(my - y,2))
        hatphi = math.atan2(my - y, mx - x)
        q = (random.normalvariate(r - hatr, self.sigmar) *
             random.normalvariate(phi - hatphi, self.sigmaphi))
        return q


    def update(self, ut, zt):
        """
        We implement the MCL algorithm from probabilistic robotics (Table 8.2)
        """
        newparticles = []
        weights = []
        for (xtm1, wtm) in self.particles:
            xtm = self.sample_motion_model(ut, xtm1)
            wtm = self.measurement_model, zt, xtm)
            newparticles.append((xtm, wtm))
            weights.append(wtm)

        weights = weights / np.sum(weights) # normalize
        
        samples = np.random.multinomial(len(self.particles), weights)

        self.particles = [newparticles[i] for i in samples]

        
def main():
    rospy.init_node('global_position_estimator')

if __name__ == '__main__':
    main()

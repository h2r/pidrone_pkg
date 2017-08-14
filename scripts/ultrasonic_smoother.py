import rospy
import numpy as np
from sensor_msgs.msg import Range

class Smoother():
    def __init__(self, pub):
        self.pub = pub
        self.mu = 0
        self.mu2 = 0
        self.sigma2 = 0
        self.alpha = 0.5
        self.rng_msg = Range()
            
    def step(self, data):
        s = data.range
        if self.reject(s):
            print 'REJECTING {} becase mu,sig is {},{}'.format(s,self.mu,np.sqrt(self.sigma2))
        else:
            self.mu = self.alpha * s + (1. - self.alpha) * self.mu
            self.mu2 = self.alpha * s**2 + (1. - self.alpha) * self.mu2
            self.sigma2 = self.mu2 - self.mu**2
            self.alpha = self.renorm(self.sigma2) # update alpha based on variance
            
            print '{}\t{}\t{}\t{}'.format(self.sigma2,self.alpha,self.mu,s)

            self.rng_msg.range = self.mu
            self.pub.publish(self.rng_msg)

    def renorm(self, sigma2):
        return 1./(1. + sigma2)

    def reject(self, s):
        sd = np.sqrt(self.sigma2)
        return np.absolute(s - self.mu)/sd > 3

if __name__ == '__main__':
    rospy.init_node('ultrasonic_smoother')
    pub = rospy.Publisher('/pidrone/ultra_smooth', Range, queue_size=1)
    smoother = Smoother(pub)
    rospy.Subscriber('/pidrone/ultrasonic', Range, smoother.step)
    rospy.spin()

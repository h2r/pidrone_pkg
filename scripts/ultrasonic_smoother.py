import rospy
import numpy as np
from sensor_msgs.msg import Range


class VarianceSmoother():
    def __init__(self, pub):
        self.pub = pub
        self.fixed_alpha = 0.1 
        self.alpha_range = np.array([0.1,0.9])
        self.alpha_range_mode = 'cutoff' # can be cutoff or scale
        self.sigma_scale = 5.

        self.p_est = 0.
        self.mu = 5.
        self.mu2 = self.mu**2 + 1.
        self.sigma2 = 0.5
        self.alpha = 0.9

        self.NR_p_est = 0.
        self.NR_mu = 5.
        self.NR_mu2 = self.mu**2 + 1.
        self.NR_sigma2 = 0.5
        self.NR_alpha = 0.9

        self.num_readings = 0
        self.num_rejects = 0
        self.rng_msg = Range()

    def update_fallback_estimate(self, s):
        self.NR_mu = self.fixed_alpha * s + (1. - self.fixed_alpha) * self.NR_mu          # estimate mu
        self.NR_mu2 = self.fixed_alpha * s**2 + (1. - self.fixed_alpha) * self.NR_mu2     # estimate mu^2
        self.NR_sigma2 = self.NR_mu2 - self.NR_mu**2 #+ 0.1 # add constant to sigma2 to prevent convergence
        self.NR_alpha = self.renorm(np.sqrt(self.NR_sigma2)) # update alpha based on variance
        self.NR_p_est = self.NR_alpha * s + (1. - self.NR_alpha) * self.NR_p_est

    def step(self, data):
        s = data.range
        self.update_fallback_estimate(s)
        # wait for some number of readings to come through so that the terms are near where they need to be
        self.num_readings += 1 
        if self.num_readings > 100 and self.reject(s):
            self.num_rejects += 1
            print 'REJECTING {} becase mu, sig is {}, {}'.format(s,self.mu,np.sqrt(self.sigma2))
        else:
            self.num_rejects = 0
            self.mu = self.fixed_alpha * s + (1. - self.fixed_alpha) * self.mu          # estimate mu
            self.mu2 = self.fixed_alpha * s**2 + (1. - self.fixed_alpha) * self.mu2     # estimate mu^2
            self.sigma2 = self.mu2 - self.mu**2 #+ 0.1 # add constant to sigma2 to prevent convergence
            self.alpha = self.renorm(np.sqrt(self.sigma2)) # update alpha based on variance
            self.p_est = self.alpha * s + (1. - self.alpha) * self.p_est
            
            # print self.alpha < self.alpha_range[0], self.sigma2 < 0
            print 's2, a, mu, new\t{}\t{}\t{}\t{}'.format(
                np.absolute(s-self.p_est)/np.sqrt(self.sigma2),self.alpha,self.mu,s)

        if self.num_rejects > 15:       # if we've rejected too many readings, snap to backup
            print 'TOO MANY REJECTS'
            self.num_rejects = 0
            self.p_est = self.NR_p_est
            self.mu = self.NR_mu
            self.mu2 = self.NR_mu2
            self.sigma2 = self.NR_sigma2
            self.alpha = self.NR_alpha

        self.rng_msg.range = self.p_est 
        self.rng_msg.header.stamp = data.header.stamp
        self.pub.publish(self.rng_msg)

    def renorm(self, sigma):
        if self.alpha_range_mode == 'cutoff':
            a = 1./(1. + sigma * self.sigma_scale)
            return min(self.alpha_range[1], max(self.alpha_range[0],a))
        elif self.alpha_range_mode == 'scale':
            return (self.alpha_range[1] - self.alpha_range[0])/(1. +
            sigma * self.sigma_scale) + self.alpha_range[0]
        else:
            print 'ERROR', self.alpha_range_mode, 'is not valid.'
            return None
    
    def reject(self, s):
        sd = np.sqrt(self.sigma2)
        return np.absolute(s - self.p_est)/sd > 3.

class OldSmoother():
    def __init__(self, pub):
        self.pub = pub
        self.s0 = 10.
        self.mu = 10.
        self.mu2 = 10.
        self.sigma2 = 0.5
        self.fixed_alpha = 0.1
        self.alpha = 0.5
        self.alpha_range = np.array([0.1,0.9])
        self.rng_msg = Range()
        self.num_readings = 0
            
    def step(self, data):
        s = data.range
        # wait for some number of readings to come through so that the terms
        # are near where they need to be
        self.num_readings += 1 
        if False and self.num_readings > 100 and self.reject(s):
            print 'REJECTING {} becase mu, sig is {}, {}'.format(s,self.mu,np.sqrt(self.sigma2))
        else:
            self.mu = self.fixed_alpha * s + (1. - self.fixed_alpha) * self.mu
            self.mu2 = self.fixed_alpha * s**2 + (1. - self.fixed_alpha) * self.mu2
            self.sigma2 = self.mu2 - self.mu**2
            self.alpha = self.renorm(self.sigma2) # update alpha based on variance
            #print 's2, a, mu, new\t{}\t{}\t{}\t{}'.format(
            #    np.absolute(s-self.s0)/np.sqrt(self.sigma2),self.alpha,self.mu,s)
            self.s0 = self.alpha * s + (1. - self.alpha) * self.s0
            self.rng_msg.range = self.s0
        
        self.rng_msg.header.stamp = rospy.get_rostime()
        self.pub.publish(self.rng_msg)

    def renorm(self, sigma2):
        return (self.alpha_range[1] - self.alpha_range[0])/(1. + sigma2) + self.alpha_range[0]
    
    def reject(self, s):
        sd = np.sqrt(self.sigma2)
        return np.absolute(s - self.s0)/sd > 5.

if __name__ == '__main__':
    rospy.init_node('ultrasonic_smoother')
    pub = rospy.Publisher('/pidrone/ultra_smooth', Range, queue_size=1)
    smoother = VarianceSmoother(pub)
    rospy.Subscriber('/pidrone/ultrasonic', Range, smoother.step)
    rospy.spin()

import argparse
import rospy
from pidrone_pkg.msg import State
import subprocess
import os


class StateEstimator(object):
    """
    This class is intended to unify the different state estimators so that the
    user only has to call this script to interact with state estimators. This
    node publishes to /pidrone/state, which offers the best state estimate
    based on whichever state estimators are to be used, depending on the
    command-line arguments that the user passes into this script.
    
    The different state estimators are:
        - EMA: uses an exponential moving average
        - UKF with a 2D state vector
        - UKF with a 7D state vector
        - UKF with a 12D state vector
        - MoCap: provides ground truth
        - Simulation (drone_simulator.py): provides simulated ground truth
        
    This script runs the state estimators in non-blocking subprocesses. When
    this script is terminating, there is a finally clause that will attempt to
    terminate the subprocesses. Note that this needs to be tested well, and that
    the shell=True argument for these subprocesses could be a security hazard
    that should be investigated further.
    """
    
    def __init__(self, primary, others):
        self.state_msg = State()
        
        self.primary_estimator = primary
        self.other_estimators = others
        
        self.process_cmds_dict = {
                'ema': 'python StateEstimators/state_estimator_ema.py',
                'ukf2d': 'python StateEstimators/state_estimator_ukf_2d.py',
                'ukf7d': 'python StateEstimators/state_estimator_ukf_7d.py',
                'ukf12d': 'python StateEstimators/state_estimator_ukf_12d.py',
                'mocap': 'python StateEstimators/state_estimator_mocap.py',  # TODO: Implement this
                'simulator': 'python drone_simulator.py --dim 1'  # TODO: Improve drone_simulator usage?
        }
        
        # List to store the process objects from subprocess.Popen()
        self.processes = []

        self.ukf_topics = {2: '/pidrone/state/ukf_2d',
                           7: '/pidrone/state/ukf_7d',
                           12: '/pidrone/state/ukf_12d'}
        self.ema_topic = '/pidrone/state/ema'
        # TODO: Get the mocap node to publish State messages to this topic
        self.mocap_topic = '/pidrone/state/mocap'
        # TODO: Get the drone_simulator to publish State messages to this topic
        self.simulator_topic = '/pidrone/state/simulator'
        
        self.start_estimator_subprocess_cmds()
        self.initialize_ros()

    def initialize_ros(self):
        node_name = os.path.splitext(os.path.basename(__file__))[0]
        rospy.init_node(node_name)

        self.state_pub = rospy.Publisher('/pidrone/state', State, queue_size=1,
                                         tcp_nodelay=False)
        rospy.spin()
    
    def start_estimator_subprocess_cmds(self):
        process_cmds = [self.process_cmds_dict[self.primary_estimator]]
        if self.primary_estimator == 'ukf2d':
            # We want the EMA to provide x and y position and velocity
            # estimates, for example, to supplement the 2D UKF's z position and
            # velocity estimates.
            process_cmds.append(self.process_cmds_dict['ema'])
            self.ema_state_msg = State()
            rospy.Subscriber(self.ema_topic, State, self.ema_helper_callback)
            rospy.Subscriber(self.ukf_topics[2], State, self.state_callback)
        elif self.primary_estimator == 'ukf7d':
            rospy.Subscriber(self.ukf_topics[7], State, self.state_callback)
        elif self.primary_estimator == 'ukf12d':
            rospy.Subscriber(self.ukf_topics[12], State, self.state_callback)
        elif self.primary_estimator == 'ema':
            rospy.Subscriber(self.ema_topic, State, self.state_callback)
        elif self.primary_estimator == 'mocap':
            rospy.Subscriber(self.mocap_topic, State, self.state_callback)
        elif self.primary_estimator == 'simulator':
            rospy.Subscriber(self.simulator_topic, State, self.state_callback)
        
        # Set up the process commands for the non-primary estimators
        if self.other_estimators is not None:
            for other_estimator in self.other_estimators:
                # Avoid running a subprocess more than once
                if other_estimator not in process_cmds:
                    process_cmds.append(self.process_cmds_dict[other_estimator])
            
        for p in process_cmds:
            print 'Starting:', p
            # NOTE: shell=True could be security hazard
            self.processes.append((p, subprocess.Popen(p, shell=True)))

    def state_callback(self, msg):
        """
        Callback that handles the primary estimator republishing.
        """
        
        # TODO: Consider creating a new State message rather than modifying just
        #       one State message
        self.state_msg.header.stamp = rospy.Time.now()
        if self.primary_estimator == 'ukf2d':
            # Use EMA data for x and y positions and velocities
            x = self.ema_state_msg.pose_with_covariance.pose.position.x
            y = self.ema_state_msg.pose_with_covariance.pose.position.y
            vel_x = self.ema_state_msg.twist_with_covariance.twist.linear.x
            vel_y = self.ema_state_msg.twist_with_covariance.twist.linear.y
        else:
            # Use primary_estimator data for x and y positions and velocities
            x = msg.pose_with_covariance.pose.position.x
            y = msg.pose_with_covariance.pose.position.y
            vel_x = msg.twist_with_covariance.twist.linear.x
            vel_y = msg.twist_with_covariance.twist.linear.y
        
        z = msg.pose_with_covariance.pose.position.z
        vel_z = msg.twist_with_covariance.twist.linear.z
        orientation = msg.pose_with_covariance.pose.orientation
        vel_angular = msg.twist_with_covariance.twist.angular
        
        self.state_msg.pose_with_covariance.pose.position.x = x
        self.state_msg.pose_with_covariance.pose.position.y = y
        self.state_msg.pose_with_covariance.pose.position.z = z
        self.state_msg.pose_with_covariance.pose.orientation = orientation
        self.state_msg.twist_with_covariance.twist.linear.x = vel_x
        self.state_msg.twist_with_covariance.twist.linear.y = vel_y
        self.state_msg.twist_with_covariance.twist.linear.z = vel_z
        self.state_msg.twist_with_covariance.twist.angular = vel_angular
        
        # Include covariances
        self.state_msg.pose_with_covariance.covariance = msg.pose_with_covariance.covariance
        self.state_msg.twist_with_covariance.covariance = msg.twist_with_covariance.covariance
        
        self.state_pub.publish(self.state_msg)
        
    def ema_helper_callback(self, msg):
        """
        When the primary estimator is the 2D UKF, populate self.ema_state_msg
        in this callback.
        """
        self.ema_state_msg.pose_with_covariance.pose.position.x = msg.pose_with_covariance.pose.position.x
        self.ema_state_msg.pose_with_covariance.pose.position.y = msg.pose_with_covariance.pose.position.y
        self.ema_state_msg.twist_with_covariance.twist.linear.x = msg.twist_with_covariance.twist.linear.x
        self.ema_state_msg.twist_with_covariance.twist.linear.y = msg.twist_with_covariance.twist.linear.y
        

def main():
    parser = argparse.ArgumentParser(description=('The state estimator node '
                'can provide state estimates using a 1D UKF (2D state vector), '
                'a 3D UKF (7D or 12D state vector), an EMA, MoCap, or '
                'simulated ground truth data. The default is the EMA. The '
                'default UKF is the UKF with 2D state vector. The primary '
                'state estimator determines what is published to '
                '/pidrone/state, except that an incomplete state estimator '
                'like the 2D UKF will also use EMA estimates to populate x and '
                'y position, for example.'))
                
    arg_choices = ['ema', 'ukf2d', 'ukf7d', 'ukf12d', 'mocap', 'simulator']
    
    parser.add_argument('--primary', '-p',
                        choices=arg_choices,
                        default='ema',
                        help='Select the primary state estimation method')
    parser.add_argument('--others', '-o',
                        choices=arg_choices,
                        nargs='+',
                        help=('Select other state estimation nodes to run '
                              'alongside the primary state estimator, e.g., '
                              'for visualization or debugging purposes'))
                              
    args = parser.parse_args()
    
    try:
        se = StateEstimator(primary=args.primary, others=args.others)
    except Exception as e:
        print e
    finally:
        # Terminate the subprocess calls. Note, however, that if Ctrl-C is
        # entered in stdin, it seems that the subprocesses also get the Ctrl-C
        # input and are terminating based on KeyboardInterrupt
        print 'Terminating subprocess calls...'
        for process_name, process in se.processes:
            print 'Terminating:', process_name
            process.terminate()
        print 'Done.'


if __name__ == "__main__":
    main()

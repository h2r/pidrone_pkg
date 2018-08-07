#!/usr/bin/env python

from numpy.random import randn # to generate some random Gaussian noise
import numpy as np
import argparse
import os
import sys
import time


class DroneSimulator(object):
    '''
    This class is intended to simulate the drone's raw sensor outputs to aid in
    the development of an Unscented Kalman Filter (UKF). It is intended to be
    inherited by DroneSimulator1D, DroneSimulator2D, or DroneSimulator3D, which
    implement the required data generation.
    
    This script can publish data in real-time to relevant ROS topics and can
    output data to CSV files, if the user wishes to analyze the data altogether
    with a script such as state_analyzer.py and state_analyzer_1d.py (NOTE:
    These scripts may be outdated).
    '''
    
    def __init__(self, publish_ros=False, save_to_csv=False, rate=1.0, dim=3):
        self.publish_ros = publish_ros
        self.save_to_csv = save_to_csv
        self.delay_rate = 1.0/rate
        self.dim = dim # number of spatial dimensions to simulate (1, 2, or 3)
        self.start_time = time.time()
        
        if not self.publish_ros and not self.save_to_csv:
            # If none of these two args are specified, default to ROS publishing
            print 'Publishing to ROS topics by default'
            self.publish_ros = True
        if self.publish_ros:
            # Importing from within a class/function may not be good Python
            # practice, but this allows for the code to be run in a non-ROS
            # environment depending on the command-line arguments given
            global rospy
            import rospy
            
            global Imu
            global Range
            global StateGroundTruth
            global Header
            global PoseStamped
            global TwistStamped
            from sensor_msgs.msg import Imu, Range
            # TODO: Create this custom message
            from pidrone_pkg.msg import StateGroundTruth
            from std_msgs.msg import Header
            from geometry_msgs.msg import PoseStamped, TwistStamped
            
            rospy.init_node('drone_simulator')
            self.init_pubs()
        if self.save_to_csv:
            global csv
            import csv
            
    def init_pubs(self):
        '''
        Initialize ROS publishers
        '''
        self.ir_pub = rospy.Publisher('/pidrone/infrared_raw', Range, queue_size=1)
        self.imu_pub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1)
        
        # Create the publisher to publish state estimates
        self.state_ground_truth_pub = rospy.Publisher('/pidrone/state_ground_truth',
                                                      StateGroundTruth,
                                                      queue_size=1,
                                                      tcp_nodelay=False)
        
    def create_dir(self, dir):
        '''
        If directory dir does not exist, create it
        '''
        if not os.path.isdir(dir):
            print 'Creating directory:', dir
            os.mkdir(dir)
        
    def create_logs_dir(self):
        '''
        If there is no logs directory in the right location, create one
        '''
        self.create_dir(self.logs_dir)
            
    def create_logs_full_dir(self):
        '''
        If there is no logs directory with dimension in the right location,
        create one
        '''
        self.create_dir(self.logs_full_dir)
        
    def check_current_working_directory(self):
        if os.path.basename(os.getcwd()) != 'scripts':
            # Then this file is not being run from the correct directory
            print ('This program must be run from the "scripts" directory in '
                  'order to save CSV log files. Exiting...')
            sys.exit()
                
    def init_csv_files(self):
        '''
        Write the log file headers to csv files
        '''
        self.check_current_working_directory()
        self.logs_dir = '../logs'
        self.logs_full_dir = os.path.join(self.logs_dir, '{}D'.format(self.dim))
        self.create_logs_dir()
        self.create_logs_full_dir()
        self.time_header = ['Seconds', 'Nanoseconds']
        
        # For each sensor type
        for _, info_dict in self.info_dicts.iteritems():
            # For each filename corresponding to the sensor
            for key, filename in info_dict['filenames'].iteritems():
                with open(os.path.join(self.logs_full_dir, filename+'.csv'), 'w') as csv_file:
                    csv_writer = csv.writer(csv_file, delimiter=' ')
                    csv_writer.writerow(self.time_header + info_dict['headers'][key])
                
    def serialize_times(self):
        '''
        Order the sample times from different sensors chronologically
        '''
        sorted_all_data = False
        self.serialized_times = []
        while not sorted_all_data:
            first_key = True
            got_nonempty_times_list = False
            for _, info_dict in self.info_dicts.iteritems():
                num = info_dict['id']
                time_list = info_dict['times']
                if time_list:
                    # List representing sample times is not empty
                    got_nonempty_times_list = True
                    next_surveyed_time_sec = time_list[0][0]
                    next_surveyed_time_nsec = time_list[0][1]
                    next_surveyed_time = next_surveyed_time_sec + next_surveyed_time_nsec*1e-9
                    if first_key or next_surveyed_time < next_time:
                        # Found a new minimum that is the next most recent time
                        first_key = False
                        next_time = next_surveyed_time
                        next_time_sec = next_surveyed_time_sec
                        next_time_nsec = next_surveyed_time_nsec
                        next_time_id = num
                        next_time_list = time_list
            if not got_nonempty_times_list:
                # All loaded datasets are empty, so it must all be sorted
                sorted_all_data = True
            
            if not sorted_all_data:
                next_time_list.pop(0)
                # Append a fileID-time pair
                self.serialized_times.append((next_time_id, (next_time_sec, next_time_nsec)))
        
    def generate_times(self, duration, time_std_dev):
        '''
        Generate sample times based on the Hz of each sensor
        '''
        for _, info_dict in self.info_dicts.iteritems():
            time_list = info_dict['times']
            hz = info_dict['hz']
            curr_time = self.start_time
            while curr_time <= (self.start_time + duration):
                curr_time += 1/(time_std_dev * randn() + hz)
                curr_sec, curr_nsec = self.float_secs_to_time_pair(curr_time)
                time_list.append([curr_sec, curr_nsec])
            del time_list[-1] # to keep times less than or equal to duration
        
    def publish_ir(self):
        '''
        Publish simulated IR measurements
        '''
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.range = self.z_pos_ir_measurement
        self.ir_pub.publish(range_msg)
        
    def init_rpy_data(self):
        # Estimated standard deviations (radians)
        self.roll_std_dev = np.deg2rad(0.5)
        self.pitch_std_dev = np.deg2rad(0.5)
        self.yaw_std_dev = np.deg2rad(1.0)
        
    def step_rpy_data(self):
        if self.dim == 3:
            self.roll = self.roll_std_dev * randn()
            self.yaw = self.yaw_std_dev * randn()
        else:
            self.roll = 0.0
            self.yaw = 0.0
        # If 2D simulation, only simulate pitch
        self.pitch = self.pitch_std_dev * randn()
        self.imu_rpy_data.append([self.roll, self.pitch, self.yaw])
            
    def init_linear_acceleration_imu_data(self):
        # Estimated standard deviations (m/s^2)
        self.x_accel_std_dev = 0.5
        self.y_accel_std_dev = 0.5
        self.z_accel_std_dev = 0.5
        if self.correlate_z_pos_and_accel:
            self.ir_z_accels.append(0.0)
            
    def step_linear_acceleration_imu_data(self):
        if self.dim >= 2:
            # For 2D and 3D simulations, simulate x acceleration
            self.x_accel = self.x_accel_std_dev * randn()
        else:
            self.x_accel = 0.0
        if self.dim == 3:
            # For 3D simulation, simulate y acceleration
            self.y_accel = self.y_accel_std_dev * randn()
        else:
            self.y_accel = 0.0
            
        if self.correlate_z_pos_and_accel:
            # Get most recent z accel from IR
            self.z_accel = self.ir_z_accels[-1]
        else:
            self.z_accel = self.z_accel_std_dev * randn()
        self.imu_accel_data.append([self.x_accel, self.y_accel, self.z_accel])
            
    
                              
    def init_x_y_yaw_velocity_data(self):
        # Assume a model with small accelerations in x and y, with some noise
        # Estimated standard deviations
        self.x_vel_std_dev = 0.02 # meters/second
        self.y_vel_std_dev = 0.02 # meters/second
        self.yaw_vel_std_dev = np.deg2rad(2.0) # radians/second
        
    def step_x_y_yaw_velocity_data(self):
        self.x_vel = self.x_vel_std_dev * randn()
        if self.dim == 3:
            self.y_vel = self.y_vel_std_dev * randn()
            self.yaw_vel = self.yaw_vel_std_dev * randn()
        else:
            self.y_vel = 0.0
            self.yaw_vel = 0.0
        self.camera_data.append([self.x_vel, self.y_vel, self.yaw_vel])
        
    def publish_x_y_yaw_vel(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = self.x_vel
        twist_msg.twist.linear.y = self.y_vel
        twist_msg.twist.angular.z = self.yaw_vel
        self.optical_flow_pub.publish(twist_msg)
    
    def write_to_csv(self, filename, times, data_list):
        with open(os.path.join(self.logs_full_dir, filename+'.csv'), 'a') as csv_file:
            csv_writer = csv.writer(csv_file, delimiter=' ')
            for num, data in enumerate(data_list):
                csv_writer.writerow(times[num] + data)
                
    def copy_times_lists(self):
        for _, info_dict in self.info_dicts.iteritems():
            info_dict['times_copy'] = list(info_dict['times'])
            
    def write_all_to_csv(self):
        # For each sensor type
        for _, info_dict in self.info_dicts.iteritems():
            # For each filename corresponding to the sensor
            for key in info_dict['filenames']:
                self.write_to_csv(filename=info_dict['filenames'][key],
                                  times=info_dict['times_copy'],
                                  data_list=info_dict['data_lists'][key])
                                      
    def create_state_ground_truth_msg(self):
        header = Header()
        secs, nsecs = self.float_secs_to_time_pair(self.curr_time)
        header.stamp.secs = secs
        header.stamp.nsecs = nsecs
        header.frame_id = 'global'
        
        pose_msg = PoseStamped()
        twist_msg = TwistStamped()
        pose_msg.header = header
        twist_msg.header = header
        
        state_ground_truth_msg = StateGroundTruth()
        state_ground_truth_msg.pose_stamped = pose_msg
        state_ground_truth_msg.twist_stamped = twist_msg
        return state_ground_truth_msg
                
    def float_secs_to_time_pair(self, t):
        '''
        Turn a float representing seconds into a secs-nsecs pair
        '''
        secs = int(t)
        nsecs = int((t - secs)*(1e9))
        return (secs, nsecs)
    
    def run_drone(self, duration):
        '''
        Start the drone sensor output
        '''
        print 'Starting simulation...'
        self.generate_data(duration=duration, time_std_dev=0.3)
        if self.save_to_csv:
            self.write_all_to_csv()
        print '\nSimulation complete.'
        
        
        
        
        
        
        
        
class DroneSimulator1D(DroneSimulator):
    
    def __init__(self, publish_ros=False, save_to_csv=False, rate=1.0):
        super(DroneSimulator1D, self).__init__(publish_ros=publish_ros,
                                               save_to_csv=save_to_csv,
                                               rate=rate,
                                               dim=1)
        self.init_info()
        self.init_state()
        self.init_std_devs()
        if self.save_to_csv:
            self.init_csv_files()
        
    def init_info(self):
        '''
        Create dictionaries to store information about sensors
        '''
        self.ir_info = {'id' : 0,
                        'hz' : 57,
                        'topic' : '/pidrone/infrared_raw',
                        'times' : [],
                        'times_copy' : [],
                        'filenames' : {'ir' : 'ir_RAW'},
                        'headers' : {'ir' : ['Range_(meters)']},
                        'data_lists' : {'ir' : []}
                        }
        self.imu_info = {'id' : 1,
                         'hz' : 26,
                         'topic' : '/pidrone/imu',
                         'times' : [],
                         'times_copy' : [],
                         'filenames' : {'accel' : 'imu_RAW'},
                         'headers' : {'accel' : ['Accel_x_body', 'Accel_y_body', 'Accel_z_body']},
                         'data_lists' : {'accel' : []}
                         }
                            
        self.info_dicts = {'ir' : self.ir_info,
                           'imu' : self.imu_info,
                           }
                           
    def init_state(self):
        '''
        Initialize the simulated drone's actual state. For this 1D simulation,
        we define the state as the drone's position and velocity along the
        vertical axis.
        '''
        self.actual_state = [0.4, 0.0] # [pos (meters), vel (m/s)]
        self.actual_accel = 0.0 # m/s^2 along the z-axis
        # To simulate random real-world disturbances:
        self.actual_accel_std_dev = 0.0001 # m/s^2
        self.curr_time = self.start_time
        
    def step_state(self, next_time_pair):
        '''
        Perform a time step and update the drone's state
        '''
        self.prev_time = self.curr_time
        self.curr_time = next_time_pair[0] + next_time_pair[1]*1e-9
        time_step = self.curr_time - self.prev_time
        # Simulating a near-zero acceleration model
        self.actual_accel = self.actual_accel_std_dev * randn()
        self.actual_state[1] += self.actual_accel * time_step
        self.actual_state[0] += self.actual_state[1] * time_step
        # Don't allow drone to go below 9 centimeters off of the ground
        if self.actual_state[0] < 0.09:
            self.actual_state[0] = 0.09
        
    def init_std_devs(self):
        self.z_accel_imu_measured_std_dev = 0.001 # meters/second^2
        # Estimated standard deviation based on IR reading measured variance
        self.z_pos_ir_measured_std_dev = np.sqrt(2.2221e-05) # meters
    
    def take_ir_sample(self):
        self.z_pos_ir_measurement = self.z_pos_ir_measured_std_dev * randn() + self.actual_state[0]
        self.ir_info['data_lists']['ir'].append([self.z_pos_ir_measurement])
        
    def take_imu_sample(self):
        '''
        Take a reading of acceleration along the z-axis
        '''
        self.z_accel_imu_measurement = self.z_accel_imu_measured_std_dev * randn() + self.actual_accel
        self.imu_info['data_lists']['accel'].append([0.0, 0.0, self.z_accel_imu_measurement])
        
    def publish_ir(self):
        '''
        Publish simulated IR measurements
        '''
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.range = self.z_pos_ir_measurement
        self.ir_pub.publish(range_msg)
        
    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.z = self.z_accel_imu_measurement
        self.imu_pub.publish(imu_msg)
        
    def publish_actual_state(self):
        '''
        Publish the current actual state from the simulation. This is
        a StateGroundTruth message containing:
            - PoseStamped
            - TwistStamped
        Note that a lot of these ROS message fields will be left empty, as the
        1D simulation does not produce information on the entire state space of
        the drone.
        '''
        state_ground_truth_msg = self.create_state_ground_truth_msg()
        state_ground_truth_msg.pose_stamped.pose.position.z = self.actual_state[0]
        state_ground_truth_msg.twist_stamped.twist.linear.z = self.actual_state[1]
        self.state_ground_truth_pub.publish(state_ground_truth_msg)
                            
    def generate_data(self, duration, time_std_dev):
        '''
        Generate data from each sensor based on each sensor's average frequency
        
        duration : approximate number of seconds for which to generate data
        time_std_dev : standard deviation to use for the noise in the time steps
        '''
        self.generate_times(duration, time_std_dev)
        self.copy_times_lists()
        self.serialize_times()
        
        # Generate sample data in order of serialized times:
        while len(self.serialized_times) > 0:
            next_sample_id, next_time_pair = self.serialized_times.pop(0)
            self.step_state(next_time_pair)
            if self.publish_ros:
                self.publish_actual_state()
            if self.publish_ros and len(self.serialized_times) > 0:
                t0 = next_time_pair[0] + next_time_pair[1]*1e-9
                print 'Duration: {0} / {1}\r'.format(round(t0-self.start_time, 4), duration),
                t1 = self.serialized_times[0][1][0] + self.serialized_times[0][1][1]*1e-9
                # Naively sleep for a certain amount of time, not taking into
                # account the amount of time required to carry out operations in
                # this loop
                rospy.sleep(self.delay_rate*(t1-t0))
            if next_sample_id == self.ir_info['id']:
                self.take_ir_sample()
                if self.publish_ros:
                    self.publish_ir()
            elif next_sample_id == self.imu_info['id']:
                self.take_imu_sample()
                if self.publish_ros:
                    self.publish_imu()
                                            
        
        
        
        
        
        
        
        
        
        
        
        
        
        
class DroneSimulator2D(DroneSimulator):
    
    def __init__(self, publish_ros=False, save_to_csv=False, rate=1.0):
        super(DroneSimulator2D, self).__init__(publish_ros=publish_ros,
                                               save_to_csv=save_to_csv,
                                               rate=rate,
                                               dim=2)
        self.init_info()
        self.init_state()
        self.init_std_devs()
        if self.save_to_csv:
            self.init_csv_files()
        if self.publish_ros:
            global tf
            import tf
            global TwistStamped
            from geometry_msgs.msg import TwistStamped
            self.optical_flow_pub = rospy.Publisher('/pidrone/plane_err',
                                                    TwistStamped,
                                                    queue_size=1)
        
    def init_info(self):
        '''
        Create dictionaries to store information about sensors
        '''
        self.ir_info = {'id' : 0,
                        'hz' : 57,
                        'topic' : '/pidrone/infrared_raw',
                        'times' : [],
                        'times_copy' : [],
                        'filenames' : {'ir' : 'ir_RAW'},
                        'headers' : {'ir' : ['Range_(meters)']},
                        'data_lists' : {'ir' : []}
                        }
        self.imu_info = {'id' : 1,
                         'hz' : 26,
                         'topic' : '/pidrone/imu',
                         'times' : [],
                         'times_copy' : [],
                         'filenames' :
                            {'accel' : 'imu_RAW',
                             'rpy' : 'roll_pitch_yaw_RAW'
                             },
                         'headers' :
                            {'accel' : ['Accel_x_body', 'Accel_y_body', 'Accel_z_body'],
                             'rpy' : ['Roll_(rad)', 'Pitch_(rad)', 'Yaw_(rad)']
                             },
                         'data_lists' :
                            {'accel' : [],
                             'rpy' : []
                             }
                         }
        self.camera_info = {'id' : 2,
                            'hz' : 20,
                            'topic' : '/pidrone/plane_err',
                            'times' : [],
                            'times_copy' : [],
                            'filenames' : {'camera' : 'x_y_yaw_velocity_RAW'},
                            'headers' : {'camera' : ['Vel_x_(m/s)', 'Vel_y_(m/s)', 'Vel_yaw_(rad/s)']},
                            'data_lists' : {'camera' : []}
                            }
                            
        self.info_dicts = {'ir' : self.ir_info,
                           'imu' : self.imu_info,
                           'camera' : self.camera_info
                           }
            
        
    def init_state(self):
        pass
        
        
class DroneSimulator3D(DroneSimulator):
    
    def __init__(self, publish_ros=False, save_to_csv=False, rate=1.0):
        super(DroneSimulator3D, self).__init__(publish_ros=publish_ros,
                                               save_to_csv=save_to_csv,
                                               rate=rate,
                                               dim=3)
        self.init_info()
        self.init_state()
        self.init_std_devs()
        if self.save_to_csv:
            self.init_csv_files()
        if self.publish_ros:
            global tf
            import tf
            global TwistStamped
            from geometry_msgs.msg import TwistStamped
            self.optical_flow_pub = rospy.Publisher('/pidrone/plane_err',
                                                    TwistStamped,
                                                    queue_size=1)
                                                    
    def init_info(self):
        '''
        Create dictionaries to store information about sensors
        '''
        self.ir_info = {'id' : 0,
                        'hz' : 57,
                        'topic' : '/pidrone/infrared_raw',
                        'times' : [],
                        'times_copy' : [],
                        'filenames' : {'ir' : 'ir_RAW'},
                        'headers' : {'ir' : ['Range_(meters)']},
                        'data_lists' : {'ir' : []}
                        }
        self.imu_info = {'id' : 1,
                         'hz' : 26,
                         'topic' : '/pidrone/imu',
                         'times' : [],
                         'times_copy' : [],
                         'filenames' :
                            {'accel' : 'imu_RAW',
                             'rpy' : 'roll_pitch_yaw_RAW'
                             },
                         'headers' :
                            {'accel' : ['Accel_x_body', 'Accel_y_body', 'Accel_z_body'],
                             'rpy' : ['Roll_(rad)', 'Pitch_(rad)', 'Yaw_(rad)']
                             },
                         'data_lists' :
                            {'accel' : [],
                             'rpy' : []
                             }
                         }
        self.camera_info = {'id' : 2,
                            'hz' : 20,
                            'topic' : '/pidrone/plane_err',
                            'times' : [],
                            'times_copy' : [],
                            'filenames' : {'camera' : 'x_y_yaw_velocity_RAW'},
                            'headers' : {'camera' : ['Vel_x_(m/s)', 'Vel_y_(m/s)', 'Vel_yaw_(rad/s)']},
                            'data_lists' : {'camera' : []}
                            }
                            
        self.info_dicts = {'ir' : self.ir_info,
                           'imu' : self.imu_info,
                           'camera' : self.camera_info
                           }
        
    def init_state(self):
        pass
        
    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        quaternion = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        imu_msg.linear_acceleration.x = self.x_accel
        imu_msg.linear_acceleration.y = self.y_accel
        imu_msg.linear_acceleration.z = self.z_accel
        self.imu_pub.publish(imu_msg)

        
def check_positive_float_duration(val):
    '''
    Function to check that the --duration command-line argument is a positive
    float.
    '''
    value = float(val)
    if value <= 0.0:
        raise argparse.ArgumentTypeError('Duration must be positive')
    return value
        
def main():
    parser = argparse.ArgumentParser(description=('Simulate the PiDrone\'s data'
                ' outputs, either by publishing to ROS topics or saving to a'
                ' CSV file.'))
    parser.add_argument('--publish_ros', '-p', action='store_true',
                        help=('Publish to ROS topics. If no -p or -s flag given,'
                              ' will default to publishing to ROS'))
    parser.add_argument('--save_csv', '-s', action='store_true',
                        help='Save data to CSV files')
    parser.add_argument('--rate', '-r', default=1.0, type=float,
                        help=('Approximate rate at which to run the simulation '
                              'if publishing to ROS (default: 1.0)'))
    parser.add_argument('--dim', '-d', default=3, type=int, choices=[1, 2, 3],
                        help=('Number of spatial dimensions in which to simulate'
                              'the drone\'s motion (default: 3)'))
    parser.add_argument('--duration', default=10.0, type=check_positive_float_duration,
                        help='Duration (seconds) of simulation')
    args = parser.parse_args()
    drone_sim_dims = [DroneSimulator1D, DroneSimulator2D, DroneSimulator3D]
    drone_sim = drone_sim_dims[args.dim-1](publish_ros=args.publish_ros,
                                           save_to_csv=args.save_csv,
                                           rate=args.rate)
    # Run the drone
    drone_sim.run_drone(duration=args.duration)
        
if __name__ == '__main__':
    main()
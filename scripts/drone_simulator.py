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
            from sensor_msgs.msg import Imu, Range
            from pidrone_pkg.msg import StateGroundTruth
            
            # disable_signals=true seems to solve the issue of KeyboardInterrupt
            # exceptions not getting raised while rospy.sleep() is occurring.
            self.node_name = os.path.splitext(os.path.basename(__file__))[0]
            rospy.init_node(self.node_name, disable_signals=True)
            self.init_pubs()
        if self.save_to_csv:
            global csv
            import csv
            
    def init_pubs(self):
        '''
        Initialize ROS publishers
        '''
        self.ir_pub = rospy.Publisher('/pidrone/infrared', Range, queue_size=1)
        self.imu_pub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1)
        
        # Create the publisher to publish state estimates
        self.state_ground_truth_pub = rospy.Publisher('/pidrone/state/ground_truth',
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
        state_ground_truth_msg = StateGroundTruth()
        state_ground_truth_msg.header.stamp = rospy.Time.now()
        state_ground_truth_msg.header.frame_id = 'global'
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
                        'topic' : '/pidrone/infrared',
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
        self.actual_accel_std_dev = 0.01 # m/s^2
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
        # Estimated standard deviation (meters) based on IR reading measured
        # variance
        if ir_var is None:
            self.z_pos_ir_measured_std_dev = np.sqrt(2.2221e-05)
        else:
            self.z_pos_ir_measured_std_dev = np.sqrt(ir_var)
    
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
            - Header
            - Pose
            - Twist
        Note that a lot of these ROS message fields will be left empty, as the
        1D simulation does not produce information on the entire state space of
        the drone.
        '''
        state_ground_truth_msg = self.create_state_ground_truth_msg()
        state_ground_truth_msg.pose.position.z = self.actual_state[0]
        state_ground_truth_msg.twist.linear.z = self.actual_state[1]
        
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
                print_str = '\rDuration: {:.4f} / {}'.format(
                                    round(self.curr_time-self.start_time, 4),
                                    duration)
                sys.stdout.write(print_str)
                sys.stdout.flush()
                t1 = self.serialized_times[0][1][0] + self.serialized_times[0][1][1]*1e-9
                # Naively sleep for a certain amount of time, not taking into
                # account the amount of time required to carry out operations in
                # this loop
                rospy.sleep(self.delay_rate*(t1-self.curr_time))
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
        if self.publish_ros:
            global tf
            import tf
            
        self.init_info()
        self.init_state()
        self.init_std_devs()
        
        if self.save_to_csv:
            self.init_csv_files()
        if self.publish_ros:
            global PoseStamped
            global TwistStamped
            from geometry_msgs.msg import PoseStamped, TwistStamped
            self.camera_pose_pub = rospy.Publisher('/pidrone/picamera/pose',
                                                    PoseStamped,
                                                    queue_size=1)
            self.optical_flow_pub = rospy.Publisher('/pidrone/picamera/twist',
                                                    TwistStamped,
                                                    queue_size=1)
        
    def init_info(self):
        '''
        Create dictionaries to store information about sensors
        '''
        self.ir_info = {'id' : 0,
                        'hz' : 57,
                        'topic' : '/pidrone/infrared',
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
        self.optical_flow_info = {'id' : 2,
                            'hz' : 80,
                            'topic' : '/pidrone/picamera/twist',
                            'times' : [],
                            'times_copy' : [],
                            'filenames' : {'optical_flow' : 'x_y_yaw_velocity_RAW'},
                            'headers' : {'optical_flow' : ['Vel_x_(m/s)', 'Vel_y_(m/s)', 'Vel_yaw_(rad/s)']},
                            'data_lists' : {'optical_flow' : []}
                            }
        self.camera_pose_info = {'id' : 3,
                            'hz' : 80,
                            'topic' : '/pidrone/picamera/pose',
                            'times' : [],
                            'times_copy' : [],
                            'filenames' : {'camera_pose' : 'x_y_yaw_RAW'},
                            'headers' : {'camera_pose' : ['Pos_x_(m)', 'Pos_y_(m)', 'Yaw_(rad)']},
                            'data_lists' : {'camera_pose' : []}
                            }
                            
        self.info_dicts = {'ir' : self.ir_info,
                           'imu' : self.imu_info,
                           'optical_flow' : self.optical_flow_info,
                           'camera_pose' : self.camera_pose_info
                           }
                           
    def init_state(self):
        '''
        Initialize the simulated drone's actual state. For this 2D simulation,
        we define the state as the drone's position and velocity in the xy-plane
        and its yaw angle and yaw velocity.
        '''
        # State is in the global frame
        # [x (meters), y (meters), yaw (rad), x_vel (m/s), y_vel (m/s), yaw_vel (rad/s)]
        self.actual_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.actual_accel_global_frame = [0.0, 0.0, 0.0] # m/s^2 along x-, y-, and z-axes (z accel should always be 0)
        # To simulate random real-world disturbances:
        # TODO: Implement this noise (perhaps make the noise optional?)
        #self.actual_accel_std_dev = 0.0001 # m/s^2
        # TEMP: No disturbances in commanded accels
        self.actual_accel_std_dev = 0 # m/s^2
        self.curr_time = self.start_time
        # Orientation quaternion:
        self.quat = tf.transformations.quaternion_from_euler(0, # roll
                                                             0, # pitch
                                                             0) # yaw
        
    def init_std_devs(self):
        self.imu_x_accel_stddev = 0.05 # meters/second^2
        self.imu_y_accel_stddev = 0.05 # meters/second^2
        self.imu_yaw_stddev = 0.02 # radians
        self.optical_flow_x_vel_stddev = 0.1 # meters/second
        self.optical_flow_y_vel_stddev = 0.1 # meters/second
        self.optical_flow_yaw_vel_stddev = 0.1 # radians/second
        self.camera_pose_x_stddev = 0.01 # meters
        self.camera_pose_y_stddev = 0.01 # meters
        self.camera_pose_yaw_stddev = 0.1 # radians
        
    def take_imu_sample(self):
        '''
        Take a reading of acceleration along the z-axis
        '''
        self.x_accel_imu_measurement = self.imu_x_accel_stddev * randn() + self.actual_accel_global_frame[0]
        self.y_accel_imu_measurement = self.imu_y_accel_stddev * randn() + self.actual_accel_global_frame[1]
        self.imu_info['data_lists']['accel'].append([self.x_accel_imu_measurement, self.y_accel_imu_measurement, 0.0])
        
    def take_optical_flow_sample(self):
        '''
        Take a reading of x velocity, y velocity, and yaw velocity from optical
        flow
        '''
        self.x_vel_optical_flow_measurement = self.optical_flow_x_vel_stddev * randn() + self.actual_state[3]
        self.y_vel_optical_flow_measurement = self.optical_flow_y_vel_stddev * randn() + self.actual_state[4]
        self.yaw_vel_optical_flow_measurement = self.optical_flow_yaw_vel_stddev * randn() + self.actual_state[5]
        self.optical_flow_info['data_lists']['optical_flow'].append([
                                        self.x_vel_optical_flow_measurement,
                                        self.y_vel_optical_flow_measurement,
                                        self.yaw_vel_optical_flow_measurement])
                                        
    def take_camera_pose_sample(self):
        '''
        Take a reading of x, y, and yaw from camera pose data
        '''
        self.x_camera_pose_measurement = self.camera_pose_x_stddev * randn() + self.actual_state[0]
        self.y_camera_pose_measurement = self.camera_pose_y_stddev * randn() + self.actual_state[1]
        self.yaw_camera_pose_measurement = self.camera_pose_yaw_stddev * randn() + self.actual_state[2]
        self.camera_pose_info['data_lists']['camera_pose'].append([
                                        self.x_camera_pose_measurement,
                                        self.y_camera_pose_measurement,
                                        self.yaw_camera_pose_measurement])
        
    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = self.x_accel_imu_measurement
        imu_msg.linear_acceleration.y = self.y_accel_imu_measurement
        imu_msg.orientation.x = self.quat[0]
        imu_msg.orientation.y = self.quat[1]
        imu_msg.orientation.z = self.quat[2]
        imu_msg.orientation.w = self.quat[3]
        self.imu_pub.publish(imu_msg)
        
    def publish_optical_flow(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = self.x_vel_optical_flow_measurement
        twist_msg.twist.linear.y = self.y_vel_optical_flow_measurement
        twist_msg.twist.angular.z = self.yaw_vel_optical_flow_measurement
        self.optical_flow_pub.publish(twist_msg)
        
    def publish_camera_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = self.x_camera_pose_measurement
        pose_msg.pose.position.y = self.y_camera_pose_measurement
        
        x,y,z,w = tf.transformations.quaternion_from_euler(0, # roll
                                            0, # pitch
                                            self.yaw_camera_pose_measurement)
        
        pose_msg.pose.orientation.x = x
        pose_msg.pose.orientation.y = y
        pose_msg.pose.orientation.z = z
        pose_msg.pose.orientation.w = w
        self.camera_pose_pub.publish(pose_msg)
        
    def publish_actual_state(self):
        '''
        Publish the current actual state from the simulation. This is
        a StateGroundTruth message containing:
            - Header
            - Pose
            - Twist
        Note that some of these ROS message fields will be populated with NaN,
        as the 2D simulation does not produce information on the entire state
        space of the drone.
        '''
        state_ground_truth_msg = self.create_state_ground_truth_msg()
        state_ground_truth_msg.pose.position.x = self.actual_state[0]
        state_ground_truth_msg.pose.position.y = self.actual_state[1]
        state_ground_truth_msg.pose.position.z = np.nan
        
        state_ground_truth_msg.pose.orientation.x = self.quat[0]
        state_ground_truth_msg.pose.orientation.y = self.quat[1]
        state_ground_truth_msg.pose.orientation.z = self.quat[2]
        state_ground_truth_msg.pose.orientation.w = self.quat[3]
        
        # x velocity, y velocity, and yaw velocity:
        state_ground_truth_msg.twist.linear.x = self.actual_state[3]
        state_ground_truth_msg.twist.linear.y = self.actual_state[4]
        state_ground_truth_msg.twist.angular.z = self.actual_state[5]
        
        # Fill the rest with NaN
        state_ground_truth_msg.twist.linear.z = np.nan
        state_ground_truth_msg.twist.angular.x = np.nan
        state_ground_truth_msg.twist.angular.y = np.nan
        
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
        
        time_step = 0.1
        actual_states_list = self.compute_states_from_cmds(time_step=time_step)
        
        # TODO: Implement sensor data generation based on actual_states_list
        # Generate sample data in order of serialized times:
        while len(self.commanded_times) > 0:
            next_time = self.commanded_times.pop(0)
            self.actual_state, self.quat = actual_states_list.pop(0)
            # For the time being, don't use different data rates. Use a set
            # data rate defined by the command time steps (non-noisy)
            self.take_imu_sample()
            self.take_camera_pose_sample()
            self.take_optical_flow_sample()
            if self.publish_ros:
                if len(self.commanded_times) > 0:
                    print_str = '\rDuration: {:.4f} / {}'.format(
                                                round(next_time, 2), duration)
                    sys.stdout.write(print_str)
                    sys.stdout.flush()
                self.publish_actual_state()
                self.publish_imu()
                self.publish_camera_pose()
                self.publish_optical_flow()
                rospy.sleep(self.delay_rate * time_step)
                    
    def yaw_cmd(self, yaw_vel, time_step):
        '''
        Command the simulated drone to perform a yawing motion
        
        :param yaw_vel: Yaw velocity in radians/second
        :param time_step: Duration for which to execute command, in seconds
        '''
        self.actual_state[5] = yaw_vel
        self.actual_state[2] += yaw_vel * time_step
        # Orientation quaternion from yaw:
        self.quat = tf.transformations.quaternion_from_euler(0, # roll
                                                             0, # pitch
                                                             self.actual_state[2]) # yaw
        #self.update_translation(time_step)
        
    def translate_body_cmd(self, body_frame_accel, time_step):
        '''
        Command the simulated drone to perform an acceleration.
        
        :param body_frame_accel: A vector specifying the drone's acceleration
        about the body-frame x and y axes
        :param time_step: A number that is the time step over which to perform
        this acceleration
        '''
        if len(body_frame_accel) == 2:
            # Put 0 as the body-frame z-axis acceleration
            body_frame_accel.append(0.0)
        elif len(body_frame_accel) == 3:
            if body_frame_accel[2] != 0:
                body_frame_accel[2] = 0.0
        else:
            raise ValueError('Body frame acceleration vector must be given in either 2 or 3 dimensions.')
            
        # Use the quaternion self.quat, which represents the drone's current
        # orientation, to rotate the body frame acceleration vector into the
        # global frame. self.quat describes rotation from the global frame to
        # the body frame, so take the quaternion's inverse by negating its
        # real component (w)
        self.quat_body_to_global = list(self.quat) # copy the quaternion object
        self.quat_body_to_global[3] = -self.quat_body_to_global[3]
        # Introduce disturbances in body frame accelerations
        body_frame_accel = [self.actual_accel_std_dev * randn() + a for a in body_frame_accel]
        accel_vector_as_quaternion = list(body_frame_accel)
        accel_vector_as_quaternion.append(0.0) # vector as a quaternion with w=0
        # Apply quaternion rotation on a vector: q*v*q', where q is the rotation
        # quaternion, v is the vector (a "pure" quaternion with w=0), and q' is
        # the conjugate of the quaternion q
        self.actual_accel_global_frame = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(self.quat_body_to_global, accel_vector_as_quaternion),
                tf.transformations.quaternion_conjugate(self.quat_body_to_global))
        # Drop the real part w=0
        self.actual_accel_global_frame = self.actual_accel_global_frame[:3]
        self.update_translation(time_step)
        
    def update_translation(self, time_step):
        # Integrate global-frame acceleration to yield change in velocity and
        # position (constant-acceleration kinematics)
        v0_x = self.actual_state[3]
        v0_y = self.actual_state[4]
        v1_x = v0_x + self.actual_accel_global_frame[0] * time_step
        v1_y = v0_y + self.actual_accel_global_frame[1] * time_step
        x1 = self.actual_state[0] + (v0_x * time_step) + (0.5 * self.actual_accel_global_frame[0] * (time_step**2))
        y1 = self.actual_state[1] + (v0_y * time_step) + (0.5 * self.actual_accel_global_frame[1] * (time_step**2))
        
        self.actual_state[0] = x1
        self.actual_state[1] = y1
        self.actual_state[3] = v1_x
        self.actual_state[4] = v1_y
        
    def compute_states_from_cmds(self, time_step=0.1):
        states_list = []
        states_list.append((list(self.actual_state), list(self.quat)))
        drone_commands = [
            {'duration' : 2,
             'cmds' : [self.translate_body_cmd],
             'args' : [{'body_frame_accel' : [0.01, 0]}]},
            {'duration' : 2,
             'cmds' : [self.translate_body_cmd],
             'args' : [{'body_frame_accel' : [0.005, 0.007]}]},
            {'duration' : 2,
             'cmds' : [self.yaw_cmd],
             'args' : [{'yaw_vel' : np.pi}]},
            {'duration' : 10,
             'cmds' : [self.yaw_cmd, self.translate_body_cmd],
             'args' : [{'yaw_vel' : 0.3}, {'body_frame_accel' : [-0.005, -0.005]}]},
            {'duration' : 5,
             'cmds' : [self.yaw_cmd],
             'args' : [{'yaw_vel' : -np.pi/2.0}]},
            {'duration' : 2,
             'cmds' : [self.translate_body_cmd],
             'args' : [{'body_frame_accel' : [0, -0.1]}]},
            ]
        
        # self.actual_state[4] = 0.5 # give initial positive y velocity
        # drone_command_loop = [
        #     {'duration' : 2,
        #      'cmds' : [self.translate_body_cmd],
        #      'args' : [{'body_frame_accel' : [0, -0.5]}]},
        #     {'duration' : 2,
        #      'cmds' : [self.translate_body_cmd],
        #      'args' : [{'body_frame_accel' : [0, 0.5]}]}
        # ]
        # drone_commands = drone_command_loop*20
        
        self.commanded_times = []
        for cmd_set in drone_commands:
            time_spent_commanding = 0
            while time_spent_commanding < cmd_set['duration']:
                for num, cmd in enumerate(cmd_set['cmds']):
                    cmd(time_step=time_step, **cmd_set['args'][num])
                states_list.append((list(self.actual_state), list(self.quat)))
                time_spent_commanding += time_step
                self.commanded_times.append(time_spent_commanding)
        return states_list
        
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
            self.optical_flow_pub = rospy.Publisher('/pidrone/picamera/twist',
                                                    TwistStamped,
                                                    queue_size=1)
                                                    
    def init_info(self):
        '''
        Create dictionaries to store information about sensors
        '''
        self.ir_info = {'id' : 0,
                        'hz' : 57,
                        'topic' : '/pidrone/infrared',
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
        self.optical_flow_info = {'id' : 2,
                            'hz' : 20,
                            'topic' : '/pidrone/picamera/twist',
                            'times' : [],
                            'times_copy' : [],
                            'filenames' : {'optical_flow' : 'x_y_yaw_velocity_RAW'},
                            'headers' : {'optical_flow' : ['Vel_x_(m/s)', 'Vel_y_(m/s)', 'Vel_yaw_(rad/s)']},
                            'data_lists' : {'optical_flow' : []}
                            }
                            
        self.info_dicts = {'ir' : self.ir_info,
                           'imu' : self.imu_info,
                           'optical_flow' : self.optical_flow_info
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
                        help=('Number of spatial dimensions in which to '
                              'simulate the drone\'s motion (default: 3)'))
    parser.add_argument('--duration', default=60.0, type=check_positive_float_duration,
                        help='Duration (seconds) of simulation (default: 60)')
    # TODO: Test out the --ir_var flag
    parser.add_argument('--ir_var', type=float,
                        help=('IR sensor variance to use in 1D simulation'))
    args = parser.parse_args()
    global ir_var
    ir_var = args.ir_var
    drone_sim_dims = [DroneSimulator1D, DroneSimulator2D, DroneSimulator3D]
    drone_sim = drone_sim_dims[args.dim-1](publish_ros=args.publish_ros,
                                           save_to_csv=args.save_csv,
                                           rate=args.rate)
    # Run the drone
    drone_sim.run_drone(duration=args.duration)
        
if __name__ == '__main__':
    main()
#!/usr/bin/env python

from numpy.random import randn # to generate some random Gaussian noise
import numpy as np
import os
import csv
import rospy
from sensor_msgs.msg import Imu, Range
from geometry_msgs.msg import TwistStamped
import tf

class DroneSimulator(object):
    '''
    This class is intended to simulate the drone's raw sensor outputs to aid in
    the development of an Unscented Kalman Filter (UKF).
    
    Currently, this script outputs data to .csv files to interface with the
    state_analyzer.py and state_analyzer_1d.py scripts, which use the
    ukf_state_estimation.py and ukf_state_estimation_1d.py scripts,
    respectively.
    
    In later stages, this class could also publish data in real-time to the
    relevant ROS topics, if the need arises.
    '''
    
    def __init__(self, publish_ros=False, save_to_csv=False, correlate_z_pos_and_accel=False, delay_rate=1.0):
        self.publish_ros = publish_ros
        self.save_to_csv = save_to_csv
        self.correlate_z_pos_and_accel = correlate_z_pos_and_accel
        self.delay_rate = delay_rate
        # NOTE: Live playback of simulated data on ROS topics may be unnecessary,
        # as in the future we could just use rosbag to collect real data and then
        # play it back.
        if self.publish_ros:
            rospy.init_node('drone_simulator')
            self.imu_pub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1)
            self.optical_flow_pub = rospy.Publisher('/pidrone/plane_err', TwistStamped, queue_size=1)
            self.ir_pub = rospy.Publisher('/pidrone/infrared', Range, queue_size=1)
        if self.save_to_csv:
            self.log_basename = '../logs'
            self.time_header = ['Seconds', 'Nanoseconds']
            self.filenames = ['ir_RAW', 'x_y_yaw_velocity_RAW', 'roll_pitch_yaw_RAW', 'imu_RAW']
            self.filenames_to_headers = {
                self.filenames[0] : ['Range_(meters)'],
                self.filenames[1] : ['Vel_x_(m/s)', 'Vel_y_(m/s)', 'Vel_yaw_(deg/s)'],
                self.filenames[2] : ['Roll_(deg)', 'Pitch_(deg)', 'Yaw_(deg)'],
                self.filenames[3] : ['Accel_x_body', 'Accel_y_body', 'Accel_z_body']}

        # Approximate samples/second of each sensor output
        self.ir_hz = 57
        if self.correlate_z_pos_and_accel:
            self.imu_hz = self.ir_hz
        else:
            self.imu_hz = 26 # RPY info published on /pidrone/state
        self.camera_hz = 20 # just a guess
        
        # Identifier enums
        self.IR = 0
        self.IMU = 1
        self.CAMERA = 2
                
    def initialize_csv_files(self):
        '''
        Write the log file headers to csv files
        '''
        for key in self.filenames_to_headers:
            with open(os.path.join(self.log_basename, key+'.csv'), 'w') as csv_file:
                csv_writer = csv.writer(csv_file, delimiter=' ')
                csv_writer.writerow(self.time_header + self.filenames_to_headers[key])
                
    def serialize_times(self):
        '''
        Order the sample times from different sensors chronologically
        '''
        sorted_all_data = False
        self.serialized_times = []
        while not sorted_all_data:
            first_key = True
            got_nonempty_times_list = False
            for num, (time_list, _) in enumerate(self.times):
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
                
    def generate_data(self, duration, time_std_dev):
        '''
        Generate data from each sensor based on each sensor's average frequency
        
        duration : approximate number of seconds for which to generate data
        time_std_dev : standard deviation to use for the noise in the time steps
        '''
        self.ir_times = []
        self.ir_data = []
        
        self.imu_times = []
        self.imu_rpy_data = []
        self.imu_accel_data = []
        
        self.camera_times = []
        self.camera_data = []
        
        # For info on how to use numpy.random.randn:
        # https://docs.scipy.org/doc/numpy-1.14.0/reference/generated/numpy.random.randn.html#numpy.random.randn
        
        if not self.correlate_z_pos_and_accel:
            self.times = ((self.ir_times, self.ir_hz),
                          (self.imu_times, self.imu_hz),
                          (self.camera_times, self.camera_hz))
            self.generate_times(duration, time_std_dev)
        else:
            # Don't generate separate times for IR
            self.times = ((self.imu_times, self.imu_hz),
                          (self.camera_times, self.camera_hz))
            self.generate_times(duration, time_std_dev)
            # Instead, set equal to the randomly generated IMU times
            self.ir_times = self.imu_times
        self.times_lists_copies = (list(self.ir_times), list(self.imu_times), list(self.camera_times))
            
        self.serialize_times()
        
        on_first_ir = True
        on_first_imu = True
        on_first_camera = True
        # Generate sample data in order of serialized times:
        while len(self.serialized_times) > 0:
            next_sample_id, next_time_pair = self.serialized_times.pop(0)
            if self.publish_ros and len(self.serialized_times) > 0:
                t0 = next_time_pair[0] + next_time_pair[1]*1e-9
                print 'Duration: {0} / {1}\r'.format(round(t0, 4), duration),
                t1 = self.serialized_times[0][1][0] + self.serialized_times[0][1][1]*1e-9
                # It might not be valid to keep updating the Rate like so and
                # expect ROS to sleep the correct amount each time
                #r = rospy.Rate(1/float(t1-t0)) # 1/dt Hz
                #r.sleep()
                # Naively sleep for a certain amount of time, not taking into
                # account the amount of time required to carry out operations in
                # this loop
                rospy.sleep(self.delay_rate*(t1-t0))
            if next_sample_id == self.IR:
                if on_first_ir:
                    on_first_ir = False
                    self.init_ir_data(next_time_pair)
                else:
                    self.step_ir_data(next_time_pair)
                if self.publish_ros:
                    self.publish_ir()
            elif next_sample_id == self.IMU:
                if on_first_imu:
                    on_first_imu = False
                    self.init_rpy_data()
                    self.init_linear_acceleration_imu_data()
                self.step_rpy_data()
                self.step_linear_acceleration_imu_data()
                if self.publish_ros:
                    self.publish_imu()
            elif next_sample_id == self.CAMERA:
                if on_first_camera:
                    on_first_camera = False
                    self.init_x_y_yaw_velocity_data()
                self.step_x_y_yaw_velocity_data()
                if self.publish_ros:
                    self.publish_x_y_yaw_vel()
                
        if self.save_to_csv:
            self.write_to_csv(filename='ir_RAW', times=self.times_lists_copies[0],
                              data_list=self.ir_data)
            self.write_to_csv(filename='roll_pitch_yaw_RAW',
                              times=self.times_lists_copies[1], data_list=self.imu_rpy_data)
            self.write_to_csv(filename='x_y_yaw_velocity_RAW',
                              times=self.times_lists_copies[2],
                              data_list=self.camera_data)
            self.write_to_csv(filename='imu_RAW',
                              times=self.times_lists_copies[1], data_list=self.imu_accel_data)
        
    def generate_times(self, duration, time_std_dev):
        # Generate sample times
        for data_time_list, data_hz in self.times:
            curr_time = 0.0
            while curr_time <= duration:
                curr_time += 1/(time_std_dev * randn() + data_hz)
                curr_sec = int(curr_time)
                curr_nsec = int((curr_time - curr_sec)*(1e9))
                data_time_list.append([curr_sec, curr_nsec])
            del data_time_list[-1] # to keep times less than or equal to duration
    
    def init_ir_data(self, first_ir_time_pair):
        # Assume a model with small accelerations, with some noise
        self.z_vel = 0.0 # meters/second
        self.z_accel_measured_std_dev = 0.001 # meters/second^2
        self.z_accel_actual_std_dev = 0.0001 # meters/second^2 # to simulate random real-world disturbances
        self.z_pos_std_dev = np.sqrt(2.2221e-05) # meters. Estimated standard deviation based on measured variance
        # Start the drone in the air
        self.curr_pos_actual = 0.4 # current position along the z-axis (meters)
        self.curr_pos_measured = self.z_pos_std_dev * randn() + self.curr_pos_actual
        self.z_accel_actual = 0.0
        self.ir_data.append([self.curr_pos_measured])
        if self.correlate_z_pos_and_accel:
            self.ir_z_accels = []
        self.next_ir_time_pair = first_ir_time_pair # sec and nsec pair
        self.next_ir_time = self.next_ir_time_pair[0] + self.next_ir_time_pair[1]*1e-9
    
    def step_ir_data(self, next_ir_time_pair):
        self.curr_ir_time = self.next_ir_time
        self.next_ir_time_pair = next_ir_time_pair # sec and nsec pair
        self.next_ir_time = self.next_ir_time_pair[0] + self.next_ir_time_pair[1]*1e-9
        time_step = self.next_ir_time - self.curr_ir_time
        # Simulating a near-zero acceleration model
        self.z_accel_actual = self.z_accel_actual_std_dev * randn()
        z_accel_measured = self.z_accel_measured_std_dev * randn() + self.z_accel_actual
        if self.correlate_z_pos_and_accel:
            self.ir_z_accels.append(z_accel_measured)
        self.z_vel += self.z_accel_actual * time_step
        self.curr_pos_actual += self.z_vel * time_step
        # Don't allow drone to go below 9 centimeters off of the ground
        if self.curr_pos_actual < 0.09:
            self.curr_pos_actual = 0.09
        self.curr_pos_measured = self.z_pos_std_dev * randn() + self.curr_pos_actual
        self.ir_data.append([self.curr_pos_measured])
        
    def publish_ir(self):
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.range = self.curr_pos_measured
        self.ir_pub.publish(range_msg)
        
    def init_rpy_data(self):
        # Estimated standard deviations (degrees)
        self.roll_std_dev = 0.5
        self.pitch_std_dev = 0.5
        self.yaw_std_dev = 1.0
        
    def step_rpy_data(self):
        self.roll = self.roll_std_dev * randn()
        self.pitch = self.pitch_std_dev * randn()
        self.yaw = self.yaw_std_dev * randn()
        self.imu_rpy_data.append([self.roll, self.pitch, self.yaw])
            
    def init_linear_acceleration_imu_data(self):
        # Estimated standard deviations (m/s^2)
        self.x_accel_std_dev = 0.5
        self.y_accel_std_dev = 0.5
        self.z_accel_std_dev = 0.5
        if self.correlate_z_pos_and_accel:
            self.ir_z_accels.append(0.0)
            
    def step_linear_acceleration_imu_data(self):
        self.x_accel = self.x_accel_std_dev * randn()
        self.y_accel = self.y_accel_std_dev * randn()
        if self.correlate_z_pos_and_accel:
            self.z_accel = self.ir_z_accels[num]
        else:
            self.z_accel = self.z_accel_std_dev * randn()
        self.imu_accel_data.append([self.x_accel, self.y_accel, self.z_accel])
            
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
                              
    def init_x_y_yaw_velocity_data(self):
        # Assume a model with small accelerations in x and y, with some noise
        # Estimates standard deviations
        self.x_vel_std_dev = 0.02 # meters/second
        self.y_vel_std_dev = 0.02 # meters/second
        self.yaw_vel_std_dev = 2.0 # degrees/second
        
    def step_x_y_yaw_velocity_data(self):
        self.x_vel = self.x_vel_std_dev * randn()
        self.y_vel = self.y_vel_std_dev * randn()
        self.yaw_vel = self.yaw_vel_std_dev * randn()
        self.camera_data.append([self.x_vel, self.y_vel, self.yaw_vel])
        
    def publish_x_y_yaw_vel(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = self.x_vel
        twist_msg.twist.linear.y = self.y_vel
        twist_msg.twist.angular.z = self.yaw_vel
        self.optical_flow_pub.publish(twist_msg)
    
    def write_to_csv(self, filename, times, data_list):
        with open(os.path.join(self.log_basename, filename+'.csv'), 'a') as csv_file:
            csv_writer = csv.writer(csv_file, delimiter=' ')
            for num, data in enumerate(data_list):
                csv_writer.writerow(times[num] + data)
    
    def run_drone(self, duration):
        '''
        Start the drone sensor output
        '''
        if self.save_to_csv:
            self.initialize_csv_files()
        self.generate_data(duration=duration, time_std_dev=0.3)
        
        
if __name__ == '__main__':
    print 'Starting simulation...'
    drone_sim = DroneSimulator(publish_ros=True)
    #drone_sim = DroneSimulator(publish_ros=True, delay_rate=100)
    #drone_sim = DroneSimulator(save_to_csv=True)
    # Run the drone for 50 seconds
    drone_sim.run_drone(duration=50)
    #drone_sim.run_drone(duration=0.5)
    print '\nSimulation complete.'
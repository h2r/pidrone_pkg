#!/usr/bin/env python

from numpy.random import randn # to generate some random Gaussian noise
import os
import csv

class DroneSimulator(object):
    '''
    This class is intended to simulate the drone's raw sensor outputs to aid in the
    development of an Unscented Kalman Filter.
    '''
    
    def __init__(self, publish_ros=False, save_to_csv=False):
        self.publish_ros = publish_ros
        self.save_to_csv = save_to_csv
        # NOTE: Live playback of simulated data on ROS topics may be unnecessary,
        # as in the future we could just use rosbag to collect real data and then
        # play it back.
        # if self.publish_ros:
        #     import rospy
        #     self.mode_pub = rospy.Publisher('/pidrone/set_mode_vel_raw', Mode, queue_size=1)
        #     self.ir_pub = rospy.Publisher('/pidrone/infrared_raw', Range, queue_size=1)
        #     self.state_pub = rospy.Publisher('/pidrone/state', State, queue_size=1)
        if self.save_to_csv:
            self.log_basename = '../logs'
            self.time_header = ['Seconds', 'Nanoseconds']
            self.filenames = ['ir_RAW', 'x_y_yaw_velocity_RAW', 'roll_pitch_yaw_RAW']
            self.filenames_to_headers = {
                self.filenames[0] : ['Range'],
                self.filenames[1] : ['Vel_x', 'Vel_y', 'Vel_yaw'],
                self.filenames[2] : ['Roll_(deg)', 'Pitch_(deg)', 'Yaw_(deg)']}

        # Approximate samples/second of each sensor output
        self.ir_hz = 57
        self.imu_hz = 26 # RPY info published on /pidrone/state
        self.camera_hz = 20 # just a guess
                
    def initialize_csv_files(self):
        '''
        Write the log file headers to csv files
        '''
        for key in self.filenames_to_headers:
            with open(os.path.join(self.log_basename, key+'.csv'), 'w') as csv_file:
                csv_writer = csv.writer(csv_file, delimiter=' ')
                csv_writer.writerow(self.time_header + self.filenames_to_headers[key])
                
    def generate_data(self, duration, time_std_dev):
        '''
        Generate data from each sensor based on each sensor's average frequency
        
        duration : approximate number of seconds for which to generate data
        time_std_dev : standard deviation to use for the noise in the time steps
        '''
        self.ir_times = []
        self.ir_data = []
        
        self.imu_times = []
        self.imu_data = []
        
        self.camera_times = []
        self.camera_data = []
        
        self.times = ((self.ir_times, self.ir_hz),
                      (self.imu_times, self.imu_hz),
                      (self.camera_times, self.camera_hz))
        
        # For info on how to use numpy.random.randn:
        # https://docs.scipy.org/doc/numpy-1.14.0/reference/generated/numpy.random.randn.html#numpy.random.randn
        
        # Generate sample times
        for data_time_list, data_hz in self.times:
            curr_time = 0.0
            while curr_time < duration:
                curr_time += 1/(time_std_dev * randn() + data_hz)
                curr_sec = int(curr_time)
                curr_nsec = int((curr_time - curr_sec)*(1e9))
                data_time_list.append([curr_sec, curr_nsec])
                
        # Generate sample data:
        self.generate_ir_data()
            
    def generate_ir_data(self):
        # Assume a model with small accelerations, with some noise
        z_vel = 0.0 # meters/second
        z_accel_std_dev = 0.01 # meters/second^2
        z_pos_std_dev = 0.005 # meters. Estimated standard deviation of 5 mm
        # Start the drone in the air
        curr_pos = 0.4 # current position along the z-axis (meters)
        next_ir_time = self.ir_times[0][0] + self.ir_times[0][1]*1e-9
        for i in range(len(self.ir_times) - 1):
            self.ir_data.append([curr_pos])
            curr_ir_time = next_ir_time
            next_ir_time_pair = self.ir_times[i+1]
            next_ir_time = next_ir_time_pair[0] + next_ir_time_pair[1]*1e-9
            time_step = next_ir_time - curr_ir_time
            z_vel += (z_accel_std_dev * randn()) * time_step
            curr_pos += z_vel * time_step
            curr_pos = z_pos_std_dev * randn() + curr_pos
            # Don't allow drone to go below 9 centimeters off of the ground
            if curr_pos < 0.09:
                curr_pos = 0.09
        if self.save_to_csv:
            self.write_to_csv(filename='ir_RAW', times=self.ir_times, data_list=self.ir_data)
    
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
        self.generate_data(duration=duration, time_std_dev=1.0)
        
        
if __name__ == '__main__':
    drone_sim = DroneSimulator(publish_ros=False, save_to_csv=True)
    # Run the drone for 50 seconds
    drone_sim.run_drone(duration=50)
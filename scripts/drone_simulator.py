#!/usr/bin/env python

from numpy.random import randn # to generate some random Gaussian noise
import os
import csv

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
    
    def __init__(self, publish_ros=False, save_to_csv=False):
        self.publish_ros = publish_ros
        self.save_to_csv = save_to_csv
        self.correlate_z_pos_and_accel = True
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
                
        # Generate sample data:
        self.generate_ir_data()
        self.generate_roll_pitch_yaw_data()
        self.generate_x_y_yaw_velocity_data()
        self.generate_linear_acceleration_imu_data()
        
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
            
    def generate_ir_data(self):
        # Assume a model with small accelerations, with some noise
        z_vel = 0.0 # meters/second
        z_accel_std_dev = 0.01 # meters/second^2
        z_pos_std_dev = 0.005 # meters. Estimated standard deviation of 5 mm
        # Start the drone in the air
        curr_pos_actual = 0.4 # current position along the z-axis (meters)
        curr_pos_measured = z_pos_std_dev * randn() + curr_pos_actual
        z_accel_actual = 0.0
        self.ir_data.append([curr_pos_measured])
        if self.correlate_z_pos_and_accel:
            self.ir_z_accels = []
        next_ir_time = self.ir_times[0][0] + self.ir_times[0][1]*1e-9
        for i in range(len(self.ir_times) - 1):
            curr_ir_time = next_ir_time
            next_ir_time_pair = self.ir_times[i+1] # sec and nsec pair
            next_ir_time = next_ir_time_pair[0] + next_ir_time_pair[1]*1e-9
            time_step = next_ir_time - curr_ir_time
            z_accel_actual = z_accel_std_dev * randn()
            z_accel_measured = z_accel_std_dev * randn() + z_accel_actual
            if self.correlate_z_pos_and_accel:
                self.ir_z_accels.append(z_accel_measured)
            z_vel += z_accel_actual * time_step
            curr_pos_actual += z_vel * time_step
            curr_pos_measured = z_pos_std_dev * randn() + curr_pos_actual
            # Don't allow drone to go below 9 centimeters off of the ground
            if curr_pos_measured < 0.09:
                curr_pos_measured = 0.09
            self.ir_data.append([curr_pos_measured])
        if self.save_to_csv:
            self.write_to_csv(filename='ir_RAW', times=self.ir_times,
                              data_list=self.ir_data)
            
    def generate_roll_pitch_yaw_data(self):
        # Estimated standard deviations (degrees)
        roll_std_dev = 0.5
        pitch_std_dev = 0.5
        yaw_std_dev = 1.0
        for _ in self.imu_times:
            roll = roll_std_dev * randn()
            pitch = pitch_std_dev * randn()
            yaw = yaw_std_dev * randn()
            self.imu_rpy_data.append([roll, pitch, yaw])
        if self.save_to_csv:
            self.write_to_csv(filename='roll_pitch_yaw_RAW',
                              times=self.imu_times, data_list=self.imu_rpy_data)
                              
    def generate_x_y_yaw_velocity_data(self):
        # Assume a model with small accelerations in x and y, with some noise
        # Estimates standard deviations
        x_vel_std_dev = 0.02 # meters/second
        y_vel_std_dev = 0.02 # meters/second
        yaw_vel_std_dev = 2.0 # degrees/second
        for _ in self.camera_times:
            x_vel = x_vel_std_dev * randn()
            y_vel = y_vel_std_dev * randn()
            yaw_vel = yaw_vel_std_dev * randn()
            self.camera_data.append([x_vel, y_vel, yaw_vel])
        if self.save_to_csv:
            self.write_to_csv(filename='x_y_yaw_velocity_RAW',
                              times=self.camera_times,
                              data_list=self.camera_data)
                              
    def generate_linear_acceleration_imu_data(self):
        # Estimated standard deviations (m/s^2)
        x_accel_std_dev = 0.5
        y_accel_std_dev = 0.5
        z_accel_std_dev = 0.5
        if self.correlate_z_pos_and_accel:
            #self.ir_z_accels.insert(0, 0.0) # insert a 0.0 acceleration to match dimension of these IMU data
            self.ir_z_accels.append(0.0)
        # if self.correlate_z_pos_and_accel:
        #     while len(self.ir_z_accels) < len(self.imu_times):
        #         self.ir_z_accels.insert(0, 0.0) # insert a 0.0 acceleration to match dimension of these IMU data
        #     while len(self.ir_z_accels) > len(self.imu_times):
        #         del self.ir_z_accels[0]
        for num, _ in enumerate(self.imu_times):
            x_accel = x_accel_std_dev * randn()
            y_accel = y_accel_std_dev * randn()
            if self.correlate_z_pos_and_accel:
                z_accel = self.ir_z_accels[num]
            else:
                z_accel = z_accel_std_dev * randn()
            self.imu_accel_data.append([x_accel, y_accel, z_accel])
        if self.save_to_csv:
            self.write_to_csv(filename='imu_RAW',
                              times=self.imu_times, data_list=self.imu_accel_data)
    
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
    drone_sim = DroneSimulator(publish_ros=False, save_to_csv=True)
    # Run the drone for 50 seconds
    drone_sim.run_drone(duration=50)
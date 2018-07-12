#!/usr/bin/env python

# The next two lines are to make plot windows appear up front as the active ones
#import matplotlib
#matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import csv
from ukf_state_estimation import DroneStateEstimation
import os
import numpy as np


'''
The purpose of this script is to analyze the state of the drone from the log
files in pidrone_pkg/logs. This script creates plots to visualize:
    - Raw sensor data
    - Ground-truth data gathered from the MoCap system
    - EMA filtered data currently running on the drone for state estimation
    - UKF (Unscented Kalman Filter) filtered data applied retroactively on raw
      data
'''

# TODO: Make plots
# TODO: Make live playback of data on plots
# TODO: Temporarily simulate noisy camera data. Derive from gound truth mocap data?

class StateAnalyzer(object):
    
    def __init__(self, include_ema=False, include_mocap=False, unit='meters'):
        self.include_ema = include_ema
        self.include_mocap = include_mocap
        self.unit = unit
        
        self.computed_ukf = False

        self.log_file_dir = '../logs/2018-06-26_test1_ir_control'
        self.log_file_dir = '../logs'
        self.filenames = ['imu_RAW',
                     'x_y_yaw_velocity_RAW',
                     #'x_y_yaw_velocity_EMA', # TODO: Uncomment if file has data
                     'ir_RAW',
                     'ir_EMA',
                     'roll_pitch_yaw_RAW',
                     'mocap']
        self.raw_data_filenames = ['imu_RAW',
                     'x_y_yaw_velocity_RAW',
                     'ir_RAW',
                     'roll_pitch_yaw_RAW']
        self.ema_data_filenames = ['ir_EMA']
                     #'x_y_yaw_velocity_EMA', # TODO: Add to list if file has data
                     
        # Store the earliest time in all loaded data sets
        self.earliest_time = None
        
        # Approximate number of seconds by which mocap data is late compared to
        # other data in the current data set
        self.mocap_time_offset = 8.7

    def load_raw_data(self):
        '''
        Load raw data from .csv and order the data chronologically
        '''
        return self.load_and_serialize_data(self.raw_data_filenames)
        
    def load_EMA_data(self):
        '''
        Load EMA filtered data from .csv and order the data chronologically
        '''
        return self.load_and_serialize_data(self.ema_data_filenames)
        
    def load_mocap_data(self):
        '''
        Load MoCap data from .csv and order the data chronologically
        '''
        return self.load_and_serialize_data(['mocap'])
        
    def load_and_serialize_data(self, csv_filename_list):
        '''
        Load data from a list of files and order the data chronologically
        '''
        loaded_data = {} # key is the filename, value is the data in the file
        for filename in csv_filename_list:
            with open(os.path.join(self.log_file_dir, filename+'.csv'), 'r') as csv_file:
                rows_list = []
                loaded_data[filename] = rows_list
                csv_reader = csv.reader(csv_file, delimiter=' ')
                is_header = True # TODO: Check this
                for row in csv_reader:
                    if not is_header:
                        rows_list.append(row)
                    else:
                        is_header = False

        sorted_all_data = False
        serialized_data = []
        on_first_rows = True
        while not sorted_all_data:
            first_key = True
            got_nonempty_file = False
            for key in loaded_data:
                if loaded_data[key]:
                    # List representing a row of data is not empty
                    got_nonempty_file = True
                    next_surveyed_time_sec = float(loaded_data[key][0][0])
                    next_surveyed_time_nsec = float(loaded_data[key][0][1])
                    next_surveyed_time = next_surveyed_time_sec + next_surveyed_time_nsec*1e-9
                    if first_key or next_surveyed_time < next_time:
                        # Found a new minimum that is the next most recent time
                        first_key = False
                        next_time = next_surveyed_time
                        next_time_key = key
            if not got_nonempty_file:
                # All loaded datasets are empty, so it must all be sorted
                sorted_all_data = True
            
            if not sorted_all_data:
                # Get and remove the most recent row of data
                new_row = loaded_data[next_time_key].pop(0)
                # Delete Seconds and Nanoseconds
                del new_row[0:2]
                # Insert time with Seconds and Nanoseconds combined
                new_row.insert(0, next_time)
                # Append a filename-data pair
                serialized_data.append((next_time_key, new_row))
            if on_first_rows:
                on_first_rows = False
                if self.earliest_time is None:
                    self.earliest_time = next_time
                elif next_time < self.earliest_time:
                    self.earliest_time = next_time
        return serialized_data
        
    # TODO: Modify for new state vector and measurement inputs
    def compute_UKF_data(self):
        '''
        Apply a UKF on raw data
        '''
        # Create a drone state estimation object to be able to store state
        # information and apply the UKF
        self.drone_state = DroneStateEstimation()
        
        # TODO: Try to implement this in the DroneStateEstimation class in
        # ukf_state_estimation.py so as to keep UKF implementation code more
        # together
        
        # TODO: Estimate yaw (is the best that we can do right now an accumulation
        # of yaw velocities from camera data?)
        
        # Lists to store UKF state estimates and times
        states_x = []
        times_t = []
        
        # List to store raw infrared slant ranges, allowing for comparison between
        # raw and filtered data
        self.raw_ir_slant_ranges = []
        self.raw_ir_slant_range_times = []
        
        # Lists to store raw IMU roll pitch yaw data
        self.raw_roll = []
        self.raw_pitch = []
        self.raw_yaw = []
        self.raw_imu_times = []
        
        # Lists to store raw x_vel, y_vel, and yaw_vel
        self.raw_x_vel = []
        self.raw_y_vel = []
        self.raw_yaw_vel = []
        self.raw_camera_times = []
        
        # List to store IMU z-accelerations
        self.raw_imu_z_accel = []
        self.raw_imu_z_accel_times = []
        
        raw_data = self.load_raw_data()
        
        ready_to_filter = False
        got_ir = False
        got_rpy = False
        got_imu = False
        while not ready_to_filter:
            # Get initial measurements in order to be able to initialize the
            # filter's state vector x and covariance matrix P.
            # Also, get first control inputs from IMU to be able to initialize
            # a start time to compute a dt between between control inputs, to be
            # used in the prediction step with the state transition function
            
            data_type, data_contents = raw_data.pop(0)
            if data_type == 'ir_RAW':
                # Got a raw slant range reading, so update the initial state
                # vector of the UKF
                self.drone_state.ukf.x[2] = float(data_contents[1])
                # Update the state covariance matrix to reflect estimated
                # measurement error. Variance of the measurement -> variance of
                # the corresponding state variable
                self.drone_state.ukf.P[2, 2] = self.drone_state.ukf.R[2, 2]
                got_ir = True
            elif data_type == 'roll_pitch_yaw_RAW':
                self.drone_state.ukf.x[6] = float(data_contents[1]) # roll
                self.drone_state.ukf.x[7] = float(data_contents[2]) # pitch
                self.drone_state.ukf.x[8] = float(data_contents[3]) # yaw
                self.drone_state.ukf.P[6, 6] = self.drone_state.ukf.R[4, 4]
                self.drone_state.ukf.P[7, 7] = self.drone_state.ukf.R[5, 5]
                self.drone_state.ukf.P[8, 8] = self.drone_state.ukf.R[6, 6]
                got_rpy = True
            elif data_type == 'imu_RAW':
                self.drone_state.last_state_transition_time = data_contents[0]
                got_imu = True
                
            if got_ir and got_rpy and got_imu:
                ready_to_filter = True
        
        # Now we are ready to filter the data. Note that the FilterPy library's
        # UnscentedKalmanFilter class requires that the predict() method be
        # called at least once before the first call to update() is made
        just_did_update = False
        for data_type, data_contents in raw_data:
            new_time = data_contents[0]
            #print self.drone_state.ukf.P
            # TODO: Get rid of the following code block
            # # -----
            # # Check for positive semidefinite matrix P, borrowed from https://stackoverflow.com/questions/5563743/check-for-positive-definiteness-or-positive-semidefiniteness/17265664#17265664
            # import scipy
            # def isPSD(A, tol=1e-8):
            #     E,V = scipy.linalg.eigh(A)
            #     return np.all(E > -tol)
            # print isPSD(self.drone_state.ukf.P)
            # # -----
            
            if data_type == 'imu_RAW':
                # Raw IMU accelerometer data is treated as the control input
                # Compute the time interval since the last state transition /
                # control input
                self.drone_state.dt = new_time - self.drone_state.last_state_transition_time
                # Set the current time at which we just received a control input
                # to be the last input time
                self.drone_state.last_state_transition_time = new_time
                
                x_accel = float(data_contents[1]) # (m/s^2)
                y_accel = float(data_contents[2]) # (m/s^2)
                z_accel = float(data_contents[3]) # (m/s^2)
                
                self.raw_imu_z_accel.append(z_accel)
                self.raw_imu_z_accel_times.append(new_time)
                self.drone_state.last_control_input = np.array([x_accel,
                                                                y_accel,
                                                                z_accel])
            # Compute the prior
            self.drone_state.ukf.predict(dt=self.drone_state.dt,
                              fx=self.drone_state.state_transition_function,
                              u=self.drone_state.last_control_input)
            self.drone_state.computed_first_prior = True
            just_did_update = False
                
            if data_type == 'ir_RAW' and self.drone_state.computed_first_prior:
                # We have just received a measurement, so compute the
                # measurement update step after having computed the new prior in
                # the prediction step
                
                # TODO: Figure out how to compute process sigmas, with a correct dt?
                if just_did_update:
                    self.drone_state.ukf.compute_process_sigmas(dt=self.drone_state.dt, u=np.array([0, 0, 0]))

                # This last measurement will be stored in self.drone_state.ukf.z
                # in the update step
                slant_range = float(data_contents[1])
                self.raw_ir_slant_ranges.append(slant_range)
                self.raw_ir_slant_range_times.append(new_time)
                measurement_z = np.array([slant_range])
                # Due to asynchronous measurement inputs, the measurement vector
                # z dynamically changes size. As a result, we must also
                # dynamically alter the UKF's measurement function hx
                
                self.drone_state.ukf.update(measurement_z,
                                    hx=self.drone_state.measurement_function_ir,
                                    R=self.drone_state.measurement_cov_ir)
                #just_did_update = True

            elif data_type == 'x_y_yaw_velocity_RAW' and self.drone_state.computed_first_prior:
                
                # TODO: Figure out how to compute process sigmas, with a correct dt?
                if just_did_update:
                    self.drone_state.ukf.compute_process_sigmas(dt=self.drone_state.dt, u=np.array([0, 0, 0]))
                    
                x_vel = float(data_contents[1])
                y_vel = float(data_contents[2])
                yaw_vel = float(data_contents[3]) # TODO: express in rad/s
                self.raw_x_vel.append(x_vel)
                self.raw_y_vel.append(y_vel)
                self.raw_yaw_vel.append(yaw_vel)
                self.raw_camera_times.append(new_time)
                measurement_z = np.array([x_vel,
                                          y_vel,
                                          yaw_vel])
                self.drone_state.ukf.update(measurement_z,
                          hx=self.drone_state.measurement_function_optical_flow,
                          R=self.drone_state.measurement_cov_optical_flow)
                #just_did_update = True
            
            elif data_type == 'roll_pitch_yaw_RAW' and self.drone_state.computed_first_prior:
                
                # TODO: Figure out how to compute process sigmas, with a correct dt?
                if just_did_update:
                    self.drone_state.ukf.compute_process_sigmas(dt=self.drone_state.dt, u=np.array([0, 0, 0]))
                    
                roll_raw = float(data_contents[1])
                pitch_raw = float(data_contents[2])
                yaw_raw = float(data_contents[3])
                self.raw_roll.append(roll_raw)
                self.raw_pitch.append(pitch_raw)
                self.raw_yaw.append(yaw_raw)
                self.raw_imu_times.append(new_time)
                measurement_z = np.array([roll_raw,
                                          pitch_raw,
                                          yaw_raw])
                self.drone_state.ukf.update(measurement_z,
                                   hx=self.drone_state.measurement_function_rpy,
                                   R=self.drone_state.measurement_cov_rpy)
                #just_did_update = True
            
            states_x.append(self.drone_state.ukf.x)
            times_t.append(new_time)
        self.computed_ukf = True
        return states_x, times_t

    def plot_altitudes(self):
        # Note that raw and EMA filtered data are slant ranges that do not account
        # for the attitude of the drone. These values are currently *treated* like
        # altitude in the drone flight code. Mo-Cap and UKF data, however, are
        # *actual* altitudes (well, UKF actually computes velocities, but here we
        # integrate to yield estimated altitudes. Of course, this is not the most
        # correct way to attain altitude values in a UKF... instead, z position
        # should be a state variable in the state vector)
        
        plt.figure()
        plt.plot(self.raw_ir_slant_range_times, self.raw_ir_slant_ranges, label='Raw IR range')
        if self.include_ema:
            plt.plot(self.ema_range_times, self.ema_ranges, label='EMA filtered IR range')
        if self.include_mocap:
            plt.plot(self.mocap_altitude_times, self.mocap_altitudes, label='Mo-Cap altitude')
        plt.plot(self.ukf_times, self.ukf_altitudes, label='UKF altitude')
        
        plt.xlabel('Time (seconds)')
        plt.ylabel('Drone altitude ({})'.format(self.unit))
        plt.title('Comparison of Drone Altitude Estimates')
        
        plt.legend()
        print 'Altitude plot created.'
        plt.show(block=False)
        #plt.draw()
        
    def plot_z_velocities(self):
        plt.figure()
        plt.plot(self.raw_ir_slant_range_times, self.raw_ir_vels, label='Raw IR velocity')
        if self.include_ema:
            plt.plot(self.ema_range_times, self.ema_vels, label='EMA filtered IR velocity')
        if self.include_mocap:
            plt.plot(self.mocap_altitude_times, self.mocap_vels, label='Mo-Cap velocity')
        plt.plot(self.ukf_times, self.ukf_z_velocities, label='UKF z-velocity')
        
        plt.xlabel('Time (seconds)')
        plt.ylabel('Z-velocity ({}/second)'.format(self.unit))
        plt.title('Comparison of Drone Z-Velocity Estimates')
        
        plt.legend()
        print 'Z-velocity plot created.'
        plt.show(block=False)
        #plt.draw()
        
    def plot_yaw_vel(self):
        plt.figure()
        plt.plot(self.raw_camera_times, self.raw_yaw_vel, label='Raw yaw velocity from camera')
        
        # TODO: Implement these plotting options
        if self.include_ema:
            pass
        if self.include_mocap:
            pass
        
        plt.plot(self.ukf_times, self.ukf_yaw_vel, label='UKF yaw velocity')
        
        plt.xlabel('Time (seconds)')
        plt.ylabel('Yaw Velocity (degrees/second)') # TODO: Is it degrees? Make into radians
        plt.title('Comparison of Drone Yaw Velocity Estimates')
        
        plt.legend()
        print 'Yaw velocity plot created.'
        plt.show(block=False)
                                
    def compare_altitudes(self, do_plot=False):
        if not self.computed_ukf:
            self.ukf_states, self.ukf_times = self.compute_UKF_data()
            self.ukf_times = [(t - self.earliest_time  + self.mocap_time_offset) for t in self.ukf_times]
        self.raw_ir_slant_range_times = [(t - self.earliest_time + self.mocap_time_offset) for t in self.raw_ir_slant_range_times]
        self.ukf_altitudes = [row[2] for row in self.ukf_states]
        if self.include_ema:
            self.get_ir_ema_altitude_data()
        if self.include_mocap:
            self.get_mocap_altitude_data()
        
        if do_plot:
            self.plot_altitudes()
                                
    # TODO: Test this method
    def compare_z_velocities(self, do_plot=False):
        # Get UKF z-velocity data
        if not self.computed_ukf:
            self.ukf_states, self.ukf_times = self.compute_UKF_data()
            self.ukf_times = [(t - self.earliest_time  + self.mocap_time_offset) for t in self.ukf_times]
        self.ukf_z_velocities = [row[5] for row in self.ukf_states]
                
        # Compute estimated velocities from raw IR sensor positions values
        self.raw_ir_slant_range_times = [(t - self.earliest_time + self.mocap_time_offset) for t in self.raw_ir_slant_range_times]
        self.raw_ir_vels = []
        for num, position in enumerate(self.raw_ir_slant_ranges):
            # Don't try to compute a velocity for the first position value
            if num != 0:
                dt = self.raw_ir_slant_range_times[num] - self.raw_ir_slant_range_times[num-1]
                dz = position - self.raw_ir_slant_ranges[num-1]
                dzdt = dz/dt
                self.raw_ir_vels.append(dzdt)
        del self.raw_ir_slant_range_times[0] # to fit dimensions of raw_ir_vels
        
        if self.include_ema:
            # Compute estimated velocities from EMA smoothed positions
            self.get_ir_ema_altitude_data()
            self.ema_vels = []
            for num, position in enumerate(self.ema_ranges):
                if num != 0:
                    dt = self.ema_range_times[num] - self.ema_range_times[num-1]
                    dz = position - self.ema_ranges[num-1]
                    dzdt = dz/dt
                    self.ema_vels.append(dzdt)
            del self.ema_range_times[0] # to fit dimensions of ema_vels
                
        if self.include_mocap:
            # Compute estimated velocities from Mo-Cap positions
            self.get_mocap_altitude_data()
            self.mocap_vels = []
            for num, position in enumerate(self.mocap_altitudes):
                if num != 0:
                    dt = self.mocap_altitude_times[num] - self.mocap_altitude_times[num-1]
                    dz = position - self.mocap_altitudes[num-1]
                    dzdt = dz/dt
                    self.mocap_vels.append(dzdt)
            del self.mocap_altitude_times[0] # to fit dimensions of mocap_vels
            
        if do_plot:
            self.plot_z_velocities()
            
    def compare_yaw_vel(self, do_plot=False):
        if not self.computed_ukf:
            self.ukf_states, self.ukf_times = self.compute_UKF_data()
            self.ukf_times = [(t - self.earliest_time  + self.mocap_time_offset) for t in self.ukf_times]
        self.raw_camera_times = [(t - self.earliest_time + self.mocap_time_offset) for t in self.raw_camera_times]
        self.ukf_yaw_vel = [row[11] for row in self.ukf_states]
        
        # TODO: Implement this plotting
        if self.include_ema:
            pass
        if self.include_mocap:
            pass
        
        if do_plot:
            self.plot_yaw_vel()

    def get_ir_ema_altitude_data(self):
        ema_ir_data = self.load_EMA_data()
        self.ema_ranges = []
        self.ema_range_times = []
        for data_type, data_contents in ema_ir_data:
            self.ema_range_times.append(data_contents[0] - self.earliest_time + self.mocap_time_offset)
            self.ema_ranges.append(float(data_contents[1]))
        
    def get_mocap_altitude_data(self):
        mocap_data = self.load_mocap_data()
        self.mocap_altitudes = []
        self.mocap_altitude_times = []
        for data_type, data_contents in mocap_data:
            self.mocap_altitude_times.append(data_contents[0] - self.earliest_time)
            self.mocap_altitudes.append(float(data_contents[3]))
        

if __name__ == '__main__':
    state_analyzer = StateAnalyzer()
    state_analyzer.compare_altitudes(do_plot=True)
    #state_analyzer.compare_z_velocities(do_plot=True)
    #state_analyzer.compare_yaw_vel(do_plot=True)
    plt.show() # to have plot window(s) stay open


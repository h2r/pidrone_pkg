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
    
    def __init__(self):

        self.log_file_dir = '../logs/2018-06-26_test1_ir_control'
        self.filenames = ['imu_RAW',
                     #'x_y_yaw_velocity_RAW', # TODO: Uncomment if file has data
                     #'x_y_yaw_velocity_EMA', # TODO: Uncomment if file has data
                     'ir_RAW',
                     'ir_EMA',
                     'roll_pitch_RAW',
                     'mocap']
        self.raw_data_filenames = ['imu_RAW',
                     #'x_y_yaw_velocity_RAW', # TODO: Uncomment if file has data
                     'ir_RAW',
                     'roll_pitch_RAW']
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
        
    def compute_UKF_data(self):
        '''
        Apply a UKF on raw data
        '''
        # Create a drone state estimation object to be able to store state
        # information and apply the UKF
        drone_state = DroneStateEstimation()
        
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
        raw_ir_slant_ranges = []
        raw_ir_slant_range_times = []
        
        raw_data = self.load_raw_data()
        for data_type, data_contents in raw_data:
            new_time = data_contents[0]
            if data_type == 'imu_RAW' and drone_state.got_roll_pitch():
                # Raw IMU accelerometer data is treated as the control input
                if drone_state.got_first_control_input:
                    # Compute the time interval since the last control input
                    drone_state.dt_control_input = (new_time -
                                                drone_state.last_control_input_time)
                # Set the current time at which we just received a control input to
                # be the last control input time
                drone_state.last_control_input_time = new_time
                drone_state.last_control_input = (
                    np.array([float(data_contents[1])*100.,   # accel x (cm/s^2)
                              float(data_contents[2])*100.,   # accel y (cm/s^2)
                              float(data_contents[3])*100.])) # accel z (cm/s^2)
                if drone_state.got_first_control_input:
                    # Compute the prior in this prediction step
                    drone_state.ukf.predict(dt=drone_state.dt_control_input,
                                            fx=drone_state.state_transition_function,
                                            u=drone_state.last_control_input)
                    drone_state.computed_first_prior = True
                else:
                    drone_state.got_first_control_input = True
            elif data_type == 'ir_RAW' and drone_state.computed_first_prior:
                # We have computed the first prior and have just received a
                # measurement, so compute the measurement update step
                if drone_state.got_first_measurement:
                    # Compute the time interval since the last control input
                    drone_state.dt_measurement = (new_time -
                                                  drone_state.last_measurement_time)
                # Set the current time at which we just received a measurement to be
                # the last measurement time
                drone_state.last_measurement_time = new_time
                # This last measurement will be stored in drone_state.ukf.z in the
                # update step
                if drone_state.got_first_measurement:
                    slant_range = float(data_contents[1])
                    raw_ir_slant_ranges.append(slant_range)
                    raw_ir_slant_range_times.append(new_time)
                    measurement_z = np.array([0.0, # measured x velocity (none)
                                              0.0, # measured y velocity (none)
                                              0.0, # measured yaw velocity (none)
                                              slant_range])
                    # In order to perform "sensor fusion", we must dynamically alter
                    # the UKF's measurement function that is to be used based on the
                    # measurement that has come in, be it IR range data or camera x
                    # velocity, y velocity, and yaw velocity
                    drone_state.ukf.update(measurement_z,
                                           hx=drone_state.measurement_function_IR,
                                           dt=drone_state.dt_measurement)
                else:
                    drone_state.got_first_measurement = True

            elif data_type == 'x_y_yaw_velocity_RAW':
                # TODO: Implement a measurement update here
                pass
            
            elif data_type == 'roll_pitch_RAW':
                # For initial simplicity, we take roll and pitch from the IMU as
                # ground truth, so update the drone's roll and pitch variables.
                # These are important as they are used in the rotation matrix in the
                # prediction step, along with a yaw estimate. We suspect that the
                # IMU implements its own filter for roll and pitch. In a later
                # iteration of this UKF, however, it may be worth including roll and
                # pitch in the state vector
                drone_state.roll = float(data_contents[1])
                drone_state.pitch = float(data_contents[2])
            
            states_x.append(drone_state.ukf.x)
            times_t.append(new_time)
        return states_x, times_t, raw_ir_slant_ranges, raw_ir_slant_range_times

    #def plot_state_vector(raw)

    def plot_altitudes(self, raw_ranges, raw_times, ema_ranges, ema_times,
                       mocap_altitudes, mocap_times, ukf_altitudes, ukf_times):
        # Note that raw and EMA filtered data are slant ranges that do not account
        # for the attitude of the drone. These values are currently *treated* like
        # altitude in the drone flight code. Mo-Cap and UKF data, however, are
        # *actual* altitudes (well, UKF actually computes velocities, but here we
        # integrate to yield estimated altitudes. Of course, this is not the most
        # correct way to attain altitude values in a UKF... instead, z position
        # should be a state variable in the state vector)
        
        plt.plot(raw_times, raw_ranges, label='Raw IR range')
        plt.plot(ema_times, ema_ranges, label='EMA filtered IR range')
        plt.plot(mocap_times, mocap_altitudes, label='Mo-Cap altitude')
        plt.plot(ukf_times, ukf_altitudes, label='UKF altitude')
        
        plt.xlabel('Time (seconds)')
        plt.ylabel('Drone altitude (cm)')
        plt.title('Comparison of Raw Data, EMA Filtered Data, Mo-Cap Ground '
                  'Truth Data, and UKF Filtered Data for Estimating Drone Altitude')
        
        plt.legend()
        print 'Altitude plot created.'
        plt.show()
    
    def plot_z_velocities(self):
        plt.plot(self.raw_ir_times, self.raw_ir_vels, label='Raw IR velocity')
        plt.plot(self.ema_times, self.ema_vels, label='EMA filtered IR velocity')
        plt.plot(self.mocap_times, self.mocap_vels, label='Mo-Cap velocity')
        plt.plot(self.ukf_times, self.ukf_z_velocities, label='UKF z-velocity')
        
        plt.xlabel('Time (seconds)')
        plt.ylabel('Z-velocity (cm/s)')
        plt.title('Comparison of Raw Data, EMA Filtered Data, Mo-Cap Ground '
                  'Truth Data, and UKF Filtered Data for Estimating Drone Z-Velocity')
        
        plt.legend()
        print 'Z-velocity plot created.'
        plt.show()
        

    def compare_altitudes(self, do_plot=False):
        ukf_states, ukf_times, raw_ir_slant_ranges, raw_ir_slant_range_times = self.compute_UKF_data()
        ukf_times = [(t - self.earliest_time  + self.mocap_time_offset) for t in ukf_times]
        raw_ir_slant_range_times = [(t - self.earliest_time + self.mocap_time_offset) for t in raw_ir_slant_range_times]
        ukf_z_velocities = [row[3] for row in ukf_states] #ukf_states[:, 3]
        ukf_curr_altitude = 0.0
        ukf_altitudes = []
        # Estimate altitudes from UKF z velocities
        for num, velocity in enumerate(ukf_z_velocities):
            # Don't try to compute a position for the first velocity value
            if num != 0:
                dt = ukf_times[num] - ukf_times[num-1]
                # Trapezoidal rule for numerical integration. Average the velocities
                dz = ((velocity + ukf_z_velocities[num-1]) / 2.0)*dt
                ukf_curr_altitude += dz
                ukf_altitudes.append(ukf_curr_altitude)
        del ukf_times[0] # to fit dimensions of ukf_altitudes
        
        ema_ranges, ema_range_times = self.get_ir_ema_altitude_data()
        mocap_altitudes, mocap_altitude_times = self.get_mocap_altitude_data()
        
        if do_plot:
            self.plot_altitudes(raw_ir_slant_ranges, raw_ir_slant_range_times,
                                ema_ranges, ema_range_times, mocap_altitudes,
                                mocap_altitude_times, ukf_altitudes, ukf_times)
                                
    def compare_z_velocities(self, do_plot=False):
        # Get UKF z-velocity data
        ukf_states, ukf_times, raw_ir_slant_ranges, self.raw_ir_times = self.compute_UKF_data()
        self.ukf_times = [(t - self.earliest_time  + self.mocap_time_offset) for t in ukf_times]
        self.ukf_z_velocities = [row[3] for row in ukf_states]
                
        # Compute estimated velocities from raw IR sensor positions values
        self.raw_ir_times = [(t - self.earliest_time + self.mocap_time_offset) for t in self.raw_ir_times]
        self.raw_ir_vels = []
        for num, position in enumerate(raw_ir_slant_ranges):
            # Don't try to compute a velocity for the first position value
            if num != 0:
                dt = self.raw_ir_times[num] - self.raw_ir_times[num-1]
                dz = position - raw_ir_slant_ranges[num-1]
                dzdt = dz/dt
                self.raw_ir_vels.append(dzdt)
        del self.raw_ir_times[0] # to fit dimensions of raw_ir_vels
        
        # Compute estimated velocities from EMA smoothed positions
        ema_ranges, self.ema_times = self.get_ir_ema_altitude_data()
        self.ema_vels = []
        for num, position in enumerate(ema_ranges):
            if num != 0:
                dt = self.ema_times[num] - self.ema_times[num-1]
                dz = position - ema_ranges[num-1]
                dzdt = dz/dt
                self.ema_vels.append(dzdt)
        del self.ema_times[0] # to fit dimensions of ema_vels
                
        # Compute estimated velocities from Mo-Cap positions
        mocap_altitudes, self.mocap_times = self.get_mocap_altitude_data()
        self.mocap_vels = []
        for num, position in enumerate(mocap_altitudes):
            if num != 0:
                dt = self.mocap_times[num] - self.mocap_times[num-1]
                dz = position - mocap_altitudes[num-1]
                dzdt = dz/dt
                self.mocap_vels.append(dzdt)
        del self.mocap_times[0] # to fit dimensions of mocap_vels
        if do_plot:
            self.plot_z_velocities()

    def get_ir_ema_altitude_data(self):
        ema_ir_data = self.load_EMA_data()
        ema_ranges = []
        ema_range_times = []
        for data_type, data_contents in ema_ir_data:
            ema_range_times.append(data_contents[0] - self.earliest_time + self.mocap_time_offset)
            ema_ranges.append(float(data_contents[1]))
        return ema_ranges, ema_range_times

    def get_mocap_altitude_data(self):
        mocap_data = self.load_mocap_data()
        mocap_altitudes = []
        mocap_altitude_times = []
        for data_type, data_contents in mocap_data:
            mocap_altitude_times.append(data_contents[0] - self.earliest_time)
            mocap_altitudes.append(float(data_contents[3]))
        return mocap_altitudes, mocap_altitude_times
        

if __name__ == '__main__':
    state_analyzer = StateAnalyzer()
    #state_analyzer.compare_altitudes()
    #state_analyzer.compare_z_velocities()


#!/usr/bin/env python

# The next two lines are to make plot windows appear up front as the active ones
#import matplotlib
#matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import csv
from ukf_state_estimation_1d import DroneStateEstimation1D
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

class StateAnalyzer1D(object):
    
    def __init__(self, include_ema=False, include_mocap=False, unit='meters'):
        self.include_ema = include_ema
        self.include_mocap = include_mocap
        self.unit = unit

        self.log_file_dir = '../logs/2018-06-26_test1_ir_control'
        self.log_file_dir = '../logs'
        self.filenames = ['ir_RAW', 'ir_EMA', 'mocap']
        self.raw_data_filenames = ['ir_RAW']
        self.ema_data_filenames = ['ir_EMA']
                     
        # Store the earliest time in all loaded data sets
        self.earliest_time = None
        
        # Approximate number of seconds by which mocap data is late compared to
        # other data in the current data set
        self.mocap_time_offset = 21.0

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
        drone_state = DroneStateEstimation1D()
        
        # TODO: Try to implement this in the DroneStateEstimation class in
        # ukf_state_estimation.py so as to keep UKF implementation code more
        # together
        
        # Lists to store UKF state estimates and times
        states_x = []
        times_t = []
        
        # List to store raw infrared slant ranges, allowing for comparison
        # between raw and filtered data
        self.raw_ir_slant_ranges = []
        self.raw_ir_slant_range_times = []
        
        raw_data = self.load_raw_data()
        for data_type, data_contents in raw_data:
            new_time = data_contents[0]
            if drone_state.got_first_measurement:
                # Compute the time interval since the last measurement
                drone_state.dt_measurement = (new_time -
                                              drone_state.last_measurement_time)
                # Compute the prior in this prediction step
                drone_state.ukf.predict(dt=drone_state.dt_measurement,
                                        fx=drone_state.state_transition_function)
            # Set the current time at which we just received a measurement to be
            # the last measurement time
            drone_state.last_measurement_time = new_time
            # In this simplified 1D case, we take the slant range to be the
            # altitude of the drone
            slant_range = float(data_contents[1])
            # This last measurement will be stored in drone_state.ukf.z in the
            # update step
            self.raw_ir_slant_ranges.append(slant_range)
            self.raw_ir_slant_range_times.append(new_time)
            if drone_state.got_first_measurement:
                measurement_z = np.array([slant_range])
                drone_state.ukf.update(measurement_z,
                                       hx=drone_state.measurement_function)
            else:
                # Use the first measurement to set the drone state
                drone_state.ukf.x = np.array([slant_range, 0])
                drone_state.got_first_measurement = True
            
            states_x.append(drone_state.ukf.x)
            times_t.append(new_time)
        return states_x, times_t

    def plot_altitudes(self):
        # Note that raw and EMA filtered data are slant ranges that do not account
        # for the attitude of the drone. These values are currently *treated* like
        # altitude in the drone flight code. Mo-Cap and UKF data, however, are
        # *actual* altitudes (well, UKF actually computes velocities, but here we
        # integrate to yield estimated altitudes. Of course, this is not the most
        # correct way to attain altitude values in a UKF... instead, z position
        # should be a state variable in the state vector)
        
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
        plt.show()
    
    def plot_z_velocities(self):
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
        plt.show()
        

    def compare_altitudes(self, do_plot=False):
        ukf_states, ukf_times = self.compute_UKF_data()
        self.ukf_times = [(t - self.earliest_time  + self.mocap_time_offset) for t in ukf_times]
        self.raw_ir_slant_range_times = [(t - self.earliest_time + self.mocap_time_offset) for t in self.raw_ir_slant_range_times]
        self.ukf_altitudes = [row[0] for row in ukf_states]
        if self.include_ema:
            self.get_ir_ema_altitude_data()
        if self.include_mocap:
            self.get_mocap_altitude_data()
        
        if do_plot:
            self.plot_altitudes()
    
    def compare_z_velocities(self, do_plot=False):
        # Get UKF z-velocity data
        ukf_states, ukf_times = self.compute_UKF_data()
        self.ukf_times = [(t - self.earliest_time  + self.mocap_time_offset) for t in ukf_times]
        self.ukf_z_velocities = [row[1] for row in ukf_states]
                
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
    state_analyzer = StateAnalyzer1D()
    state_analyzer.compare_altitudes(do_plot=True)
    #state_analyzer.compare_z_velocities(do_plot=True)


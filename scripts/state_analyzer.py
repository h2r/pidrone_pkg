#!/usr/bin/env python

import matplotlib.pyplot as plt
import csv
from ukf_state_estimation import DroneStateEstimation


'''
The purpose of this script is to analyze the state of the drone from the log
files in pidrone_pkg/logs. This script creates plots to visualize:
    - Raw sensor data
    - Ground-truth data gathered from the MoCap system
    - EMA filtered data currently running on the drone for state estimation
    - UKF (Unscented Kalman Filter) filtered data applied retroactively on raw
      data
'''

log_file_dir = '../logs/2018-06-26_test1_ir_control'
filenames = ['imu_RAW',
             #'x_y_yaw_velocity_RAW', # TODO: Uncomment if file has data
             #'x_y_yaw_velocity_EMA', # TODO: Uncomment if file has data
             'ir_RAW',
             'ir_EMA',
             'roll_pitch_RAW',
             'mocap']
raw_data_filenames = ['imu_RAW',
             #'x_y_yaw_velocity_RAW', # TODO: Uncomment if file has data
             'ir_RAW',
             'roll_pitch_RAW']
ema_data_filenames = ['ir_EMA']
             #'x_y_yaw_velocity_EMA', # TODO: Add to list if file has data

def load_raw_data():
    '''
    Load raw data from .csv and order the data chronologically
    '''
    return load_and_serialize_data(raw_data_filenames)
    
def load_EMA_data():
    '''
    Load EMA filtered data from .csv and order the data chronologically
    '''
    return load_and_serialize_data(ema_data_filenames)
    
def load_mocap_data():
    '''
    Load MoCap data from .csv and order the data chronologically
    '''
    return load_and_serialize_data(['mocap'])
    
def load_and_serialize_data(csv_filename_list):
    '''
    Load data from a list of files and order the data chronologically
    '''
    loaded_data = {} # key is the filename, value is the data in the file
    for filename in csv_filename_list:
        with open(os.path.join(log_file_dir, filename+'.csv'), 'r') as csv_file:
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
    while not sorted_all_data:
        first_key = True
        got_nonempty_file = False
        for key in loaded_data:
            if loaded_data[key]:
                got_nonempty_file = True
                next_surveyed_time_sec = loaded_data[key][0][0]
                next_surveyed_time_nsec = loaded_data[key][0][1]
                next_surveyed_time = next_time_sec + next_time_nsec*1e-9
                if first_key or next_surveyed_time < next_time:
                    # Found a new minimum that is the next most recent time
                    first_key = False
                    next_time = next_surveyed_time
                    next_time_key = key
        if not got_nonempty_file:
            # All loaded datasets are empty, so it must all be sorted
            sorted_all_data = True
        
        # Get and remove the most recent row of data
        new_row = loaded_data[next_time_key].pop(0)
        # Delete Seconds and Nanoseconds
        del new_row[0:2]
        # Insert time with Seconds and Nanoseconds combined
        new_row.insert(0, next_time)
        # Append a filename-data pair
        serialized_data.append((next_time_key, new_row))
    return serialized_data
    
def compute_UKF_data():
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
    
    raw_data = load_raw_data()
    for data_type, data_contents in raw_data:
        if data_type == 'imu_RAW' and drone_state.got_roll_pitch():
            # Raw IMU accelerometer data is treated as the control input
            if drone_state.got_first_control_input:
                # Compute the time interval since the last control input
                drone_state.dt_control_input = (data_contents[0] -
                                            drone_state.last_control_input_time)
            # Set the current time at which we just received a control input to
            # be the last control input time
            drone_state.last_control_input_time = data_contents[0]
            drone_state.last_control_input = (
                np.array([[data_contents[1]],   # accel x
                          [data_contents[2]],   # accel y
                          [data_contents[3]]])) # accel z
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
                drone_state.dt_measurement = (data_contents[0] -
                                              drone_state.last_measurement_time)
            # Set the current time at which we just received a measurement to be
            # the last measurement time
            drone_state.last_measurement_time = data_contents[0]
            # This last measurement will be stored in drone_state.ukf.z in the
            # update step
            if drone_state.got_first_measurement:
                measurement_z = np.array(
                                    [[0.0], # measured x velocity (none)
                                     [0.0], # measured y velocity (none)
                                     [0.0], # measured yaw velocity (none)
                                     [data_contents[1]]]) # measured slant range
                # In order to perform "sensor fusion", we must dynamically alter
                # the UKF's measurement function that is to be used based on the
                # measurement that has come in, be it IR range data or camera x
                # velocity, y velocity, and yaw velocity
                drone_state.ukf.update(measurement_z,
                                       hx=drone_state.measurement_function_IR,
                                       dt=drone_state.dt_measurement)

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
            drone_state.roll = data_contents[1]
            drone_state.pitch = data_contents[2]

#def plot_state_vector(raw)

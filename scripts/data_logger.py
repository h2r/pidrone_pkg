#!/usr/bin/env python

# ROS imports
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from pidrone_pkg.msg import Mode, State
from geometry_msgs.msg import PoseStamped

# Other imports
from Queue import Queue
import csv
import os
#import argparse # TODO: allow command-line arguments for logging from drone and/or MoCap, for output filename, etc.


'''
This script allows for data to be logged from the drone and from Motion Capture
by subscribing to ROS topics and outputting data to .csv files. It is intended
to be run on a base station on the same network as the drone and the Motion
Capture system in order to be able to communicate via ROS and log streams of
data.

Working with logged data from the drone will enable realistic simulation of an
Unscented Kalman Filter (UKF) before being directly implemented in the drone
flight code. That way, by comparing logged state estimates from the drone and
the Motion Capture system with filtered state estimates using a UKF, we will be
able to test the performance of a UKF rapidly in simulation.
'''

drone_name = 'aarondrone' # TODO: make this a modifiable parameter
log_basename = '../logs'

log_queue = Queue()

time_header = ['Seconds', 'Nanoseconds']

filenames = ['imu_RAW', 'x_y_yaw_velocity_RAW', 'x_y_yaw_velocity_EMA' 'ir_RAW',
             'ir_EMA' 'roll_pitch_RAW', 'mocap']
filenames_to_headers = {filenames[0] : ['Accel_x_body', 'Accel_y_body',
                                        'Accel_z_body'],
                        filenames[1] : ['Vel_x', 'Vel_y', 'Vel_yaw'],
                        filenames[2] : ['Vel_x', 'Vel_y', 'Vel_yaw'],
                        filenames[3] : ['Range'],
                        filenames[4] : ['Range'],
                        filenames[5] : ['Roll_(deg)', 'Pitch_(deg)'],
                        filenames[6] : ['x', 'y', 'z', 'Roll', 'Pitch', 'Yaw']} # TODO: Figure out units of MoCap data

def imu_data_callback(data):
    '''
    Process IMU data:
        - x linear acceleration
        - y linear acceleration
        - z linear acceleration
    '''
    new_row = [data.header.stamp.secs,
               data.header.stamp.nsecs,
               data.linear_acceleration.x,
               data.linear_acceleration.y,
               data.linear_acceleration.z]
    log_queue.put([filenames[0], new_row])

def raw_x_y_yaw_velocity_data_callback(data):
    '''
    Process x velocity, y velocity, and yaw velocity data
    '''
    new_row = [data.header.stamp.secs,
               data.header.stamp.nsecs,
               data.x_velocity,
               data.y_velocity,
               data.yaw_velocity]
    log_queue.put([filenames[1], new_row])
    
def x_y_yaw_velocity_data_callback(data):
    '''
    Process x velocity, y velocity, and yaw velocity data
    '''
    new_row = [data.header.stamp.secs,
               data.header.stamp.nsecs,
               data.x_velocity,
               data.y_velocity,
               data.yaw_velocity]
    log_queue.put([filenames[2], new_row])

def raw_ir_data_callback(data):
    '''
    Process infrared range data
    '''
    new_row = [data.header.stamp.secs,
               data.header.stamp.nsecs,
               data.range]
    log_queue.put([filenames[3], new_row])

def ir_data_callback(data):
    '''
    Process infrared range data
    '''
    new_row = [data.header.stamp.secs,
               data.header.stamp.nsecs,
               data.range]
    log_queue.put([filenames[4], new_row])

def state_data_callback(data):
    '''
    Process roll and pitch data
    '''
    new_row = [data.header.stamp.secs,
               data.header.stamp.nsecs,
               data.roll,
               data.pitch]
    log_queue.put([filenames[5], new_row])

def mocap_data_callback(data):
    '''
    Process the Motion Capture "ground truth" data
    '''
    euler_angles = tf.transformations.euler_from_quaternion([data.pose.orientation.x,
                                                            data.pose.orientation.y,
                                                            data.pose.orientation.z,
                                                            data.pose.orientation.w])
    roll = euler_angles[0]
    pitch = euler_angles[1]
    yaw = euler_angles[2]
    new_row = [data.header.stamp.secs,
               data.header.stamp.nsecs,
               data.pose.position.z*100, # drone x
               data.pose.position.x*100, # drone y
               data.pose.position.y*100, # drone z
               roll,
               pitch,
               yaw]
    log_queue.put([filenames[6], new_row])

def initialize_csv_files():
    '''
    Write the log file headers to csv files
    '''
    for key in filenames_to_headers:
        with open(os.path.join(log_basename, key+'.csv'), 'w') as csv_file:
            csv_writer = csv.writer(csv_file, delimiter=' ')
            csv_writer.writerow(time_header + filenames_to_headers[key])

def csv_writer():
    while not rospy.is_shutdown():
        # Wait here until we receive data to log
        log_data = log_queue.get(block=True)
        with open(os.path.join(log_basename, log_data[0]+'.csv'), 'a') as csv_file:
            csv_writer = csv.writer(csv_file, delimiter=' ')
            csv_writer.writerow(log_data[1])

def listener():
    rospy.init_node('data_logger')
    
    # Subscribe to topics to which the drone publishes in order to get data
    # about the drone's state. We want both raw data and data that the drone is
    # smoothing with an EMA
    rospy.Subscriber('/pidrone/imu', Imu, imu_data_callback)
    rospy.Subscriber('/pidrone/set_mode_vel_raw', Mode, raw_x_y_yaw_velocity_data_callback)
    rospy.Subscriber('/pidrone/set_mode_vel', Mode, x_y_yaw_velocity_data_callback)
    rospy.Subscriber('/pidrone/infrared_raw', Range, raw_ir_data_callback)
    rospy.Subscriber('/pidrone/infrared', Range, ir_data_callback)
    rospy.Subscriber('/pidrone/state', State, state_data_callback)
    
    # Subscribe to the topic to which the Motion Capture system publishes in
    # order to get ground-truth data about the drone's state
    rospy.Subscriber('/vrpn_client_node/' + drone_name + '/pose', PoseStamped, mocap_data_callback)


if __name__ == '__main__':
    listener()
    initialize_csv_files()
    csv_writer()

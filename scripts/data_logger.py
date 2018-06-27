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

class DataLogger(object):
    
    def __init__(self):
        self.drone_name = 'aarondrone' # TODO: make this a modifiable parameter
        self.log_basename = '../logs'

        self.log_queue = Queue()
        self.queue_length_for_csv_writing = 10000

        self.time_header = ['Seconds', 'Nanoseconds']

        self.filenames = ['imu_RAW', 'x_y_yaw_velocity_RAW', 'x_y_yaw_velocity_EMA',
                     'ir_RAW', 'ir_EMA', 'roll_pitch_RAW', 'mocap']
        self.filenames_to_headers = {self.filenames[0] : ['Accel_x_body', 'Accel_y_body',
                                                'Accel_z_body'],
                                self.filenames[1] : ['Vel_x', 'Vel_y', 'Vel_yaw'],
                                self.filenames[2] : ['Vel_x', 'Vel_y', 'Vel_yaw'],
                                self.filenames[3] : ['Range'],
                                self.filenames[4] : ['Range'],
                                self.filenames[5] : ['Roll_(deg)', 'Pitch_(deg)'],
                                self.filenames[6] : ['x', 'y', 'z', 'Roll', 'Pitch', 'Yaw']} # TODO: Figure out units of MoCap data
        # Create a queue for each callback
        self.queues_list = []
        for filename in self.filenames:
            self.queues_list.append(Queue())

    def imu_data_callback(self, data):
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
        self.log_queue.put([self.filenames[0], new_row])
        self.queues_list[0].put(new_row)
        if self.queues_list[0].qsize() > self.queue_length_for_csv_writing:
            self.write_to_csv(self.filenames[0], self.queues_list[0])

    def raw_x_y_yaw_velocity_data_callback(self, data):
        '''
        Process x velocity, y velocity, and yaw velocity data
        '''
        new_row = [data.header.stamp.secs,
                   data.header.stamp.nsecs,
                   data.x_velocity,
                   data.y_velocity,
                   data.yaw_velocity]
        self.log_queue.put([self.filenames[1], new_row])
        self.queues_list[1].put(new_row)
        if self.queues_list[1].qsize() > self.queue_length_for_csv_writing:
            self.write_to_csv(self.filenames[1], self.queues_list[1])
        
    def x_y_yaw_velocity_data_callback(self, data):
        '''
        Process x velocity, y velocity, and yaw velocity data
        '''
        new_row = [data.header.stamp.secs,
                   data.header.stamp.nsecs,
                   data.x_velocity,
                   data.y_velocity,
                   data.yaw_velocity]
        self.log_queue.put([self.filenames[2], new_row])
        self.queues_list[2].put(new_row)
        if self.queues_list[2].qsize() > self.queue_length_for_csv_writing:
            self.write_to_csv(self.filenames[2], self.queues_list[2])

    def raw_ir_data_callback(self, data):
        '''
        Process infrared range data
        '''
        new_row = [data.header.stamp.secs,
                   data.header.stamp.nsecs,
                   data.range]
        self.log_queue.put([self.filenames[3], new_row])
        self.queues_list[3].put(new_row)
        if self.queues_list[3].qsize() > self.queue_length_for_csv_writing:
            self.write_to_csv(self.filenames[3], self.queues_list[3])

    def ir_data_callback(self, data):
        '''
        Process infrared range data
        '''
        new_row = [data.header.stamp.secs,
                   data.header.stamp.nsecs,
                   data.range]
        self.log_queue.put([self.filenames[4], new_row])
        self.queues_list[4].put(new_row)
        if self.queues_list[4].qsize() > self.queue_length_for_csv_writing:
            self.write_to_csv(self.filenames[4], self.queues_list[4])

    def state_data_callback(self, data):
        '''
        Process roll and pitch data
        '''
        new_row = [data.header.stamp.secs,
                   data.header.stamp.nsecs,
                   data.roll,
                   data.pitch]
        self.log_queue.put([self.filenames[5], new_row])
        self.queues_list[5].put(new_row)
        if self.queues_list[5].qsize() > self.queue_length_for_csv_writing:
            self.write_to_csv(self.filenames[5], self.queues_list[5])

    def mocap_data_callback(self, data):
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
        self.log_queue.put([self.filenames[6], new_row])
        self.queues_list[6].put(new_row)
        if self.queues_list[6].qsize() > self.queue_length_for_csv_writing:
            self.write_to_csv(self.filenames[6], self.queues_list[6])
            
    def write_to_csv(self, filename, queue_object, log_until_empty=False):
        with open(os.path.join(self.log_basename, filename+'.csv'), 'a') as csv_file:
            csv_writer = csv.writer(csv_file, delimiter=' ')
            if log_until_empty:
                num_rows_to_write = queue_object.qsize()
                print 'Queue size:', num_rows_to_write
            else:
                num_rows_to_write = self.queue_length_for_csv_writing
            for _ in range(num_rows_to_write):
                # Write num_rows_to_write rows from the queue
                csv_writer.writerow(queue_object.get(block=True))

    def initialize_csv_files(self):
        '''
        Write the log file headers to csv files
        '''
        for key in self.filenames_to_headers:
            with open(os.path.join(self.log_basename, key+'.csv'), 'w') as csv_file:
                csv_writer = csv.writer(csv_file, delimiter=' ')
                csv_writer.writerow(self.time_header + self.filenames_to_headers[key])

    # def csv_writer(self):
    #     while not rospy.is_shutdown():
    #         # Wait here until we receive data to log
    #         log_data = self.log_queue.get(block=True)
    #         with open(os.path.join(self.log_basename, log_data[0]+'.csv'), 'a') as csv_file:
    #             csv_writer = csv.writer(csv_file, delimiter=' ')
    #             csv_writer.writerow(log_data[1])

    def listener(self):
        rospy.init_node('data_logger')
        
        # Subscribe to topics to which the drone publishes in order to get data
        # about the drone's state. We want both raw data and data that the drone is
        # smoothing with an EMA
        rospy.Subscriber('/pidrone/imu', Imu, self.imu_data_callback)
        rospy.Subscriber('/pidrone/set_mode_vel_raw', Mode, self.raw_x_y_yaw_velocity_data_callback)
        rospy.Subscriber('/pidrone/set_mode_vel', Mode, self.x_y_yaw_velocity_data_callback)
        rospy.Subscriber('/pidrone/infrared_raw', Range, self.raw_ir_data_callback)
        rospy.Subscriber('/pidrone/infrared', Range, self.ir_data_callback)
        rospy.Subscriber('/pidrone/state', State, self.state_data_callback)
        
        # Subscribe to the topic to which the Motion Capture system publishes in
        # order to get ground-truth data about the drone's state
        rospy.Subscriber('/vrpn_client_node/' + self.drone_name + '/pose', PoseStamped, self.mocap_data_callback)


if __name__ == '__main__':
    data_logger = DataLogger()
    data_logger.listener()
    print 'Initializing .csv files...'
    data_logger.initialize_csv_files()
    #data_logger.csv_writer()
    try:
        # Wait until node is halted
        rospy.spin()
    finally:
        # Upon termination of this script, we want to log the remaining data in
        # the queues
        print 'Finishing up logging by emptying queues...'
        for num, queue_object in enumerate(data_logger.queues_list):
            data_logger.write_to_csv(data_logger.filenames[num], queue_object, log_until_empty=True)
        print 'Done logging.'
            

#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Imu, Range
from pidrone_pkg.msg import State
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

# Other imports
import numpy as np
import matplotlib.pyplot as plt
from Queue import Queue


class StateGrapher(object):
    
    def __init__(self):
        # Initialize Queue objects to allow plotting to occur in main thread
        self.state_queue = Queue()
        self.ir_data_queue = Queue()
        self.setup_plots()
        
        # Lists to store data for plotting
        self.state_times = []
        self.state_data = []
        self.ir_times = []
        self.ir_data = []
        
        self.initialize_ros()
        
    def initialize_ros(self):
        '''
        Initialize ROS-related objects, e.g., the node, subscribers, etc.
        '''
        self.node_name = 'state_grapher'
        print 'Initializing {} node...'.format(self.node_name)
        rospy.init_node(self.node_name)
        
        # To be able to plot state estimates:
        rospy.Subscriber('/pidrone/state', State, self.state_callback)
        
        # To be able to plot raw sensor data:
        rospy.Subscriber('/pidrone/infrared', Range, self.ir_data_callback) # TODO: infrared or raw_infrared?
        # TODO: Later, add Imu and TwistStamped (optical flow) data support
        
    def setup_plots(self):
        #self.fig, self.axes = plt.subplots(1, 2)
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.time_range = 10 # seconds to show on the x-axis
        
    def state_callback(self, data):
        self.state_queue.put(data)
        
    def ir_data_callback(self, data):
        self.ir_data_queue.put(data)
        
    def update_plot_state(self, msg):
        '''
        Update the plot(s) with state data. For the time being, only plotting
        position along the z-axis.
        '''
        secs = msg.pose_with_covariance_stamped.header.stamp.secs
        nsecs = msg.pose_with_covariance_stamped.header.stamp.nsecs
        new_time = secs + nsecs*1e-9
        if self.state_times:
            # If we already have data, check if we should delete old data
            first_time = self.state_times[0]
            if (new_time - first_time) > self.time_range:
                del self.state_times[0]
                del self.state_data[0]
        self.state_times.append(new_time)
        z_pos = msg.pose_with_covariance_stamped.pose.pose.position.z
        self.state_data.append(z_pos)
        
        if len(self.state_times) == 1:
            x_max = self.state_times[0] + self.time_range
        else:
            x_max = new_time
        self.update_plot(self.state_times[0], x_max)
        
    def update_plot_ir(self, msg):
        '''
        Update the plot(s) with IR data.
        '''
        secs = msg.header.stamp.secs
        nsecs = msg.header.stamp.nsecs
        new_time = secs + nsecs*1e-9
        if self.ir_times:
            # If we already have data, check if we should delete old data
            first_time = self.ir_times[0]
            if (new_time - first_time) > self.time_range:
                del self.ir_times[0]
                del self.ir_data[0]
        self.ir_times.append(new_time)
        z_pos = msg.range
        self.ir_data.append(z_pos)
        
        if len(self.ir_times) == 1:
            x_max = self.ir_times[0] + self.time_range
        else:
            x_max = new_time
        self.update_plot(self.ir_times[0], x_max)
        
    def update_plot(self, x_min, x_max):
        # Plot the data
        self.ax.plot(self.ir_times, self.ir_data)
        self.ax.plot(self.state_times, self.state_data)
        self.ax.set_xlim(x_min, x_max)
        #self.ax.draw()
        self.fig.canvas.draw_idle()
        
    def run(self):
        r = rospy.Rate(30) # Hz
        while not rospy.is_shutdown():
            if not self.state_queue.empty():
                # Update plot with state data
                self.update_plot_state(self.state_queue.get())
            if not self.ir_data_queue.empty():
                # Update plot with raw IR data
                self.update_plot_ir(self.ir_data_queue.get())
            r.sleep()
        
        
def main():
    sg = StateGrapher()
    try:
        # Wait until node is halted
        #rospy.spin()
        sg.run()
    finally:
        # Upon termination of this script, print out a helpful message
        print '{} node terminating.'.format(sg.node_name)
        
if __name__ == '__main__':
    main()
        
    
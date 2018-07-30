#!/usr/bin/env python

# ROS imports
# import rospy
# from sensor_msgs.msg import Imu, Range
# from pidrone_pkg.msg import State
# from geometry_msgs.msg import TwistStamped
# from std_msgs.msg import Header
import roslibpy

# Other imports
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
 

class StateGrapher(object):
    
    def __init__(self, hostname):
        self.hostname = hostname
        self.setup_plots()
        
        # Lists to store data for plotting
        self.state_times = []
        self.state_data = []
        self.ir_times = []
        self.ir_data = []
        
        self.initialize_ros()
        
    def initialize_ros(self):
        '''
        Initialize ROS-related objects, e.g., the connection to the ROS master,
        the subscribers, etc.
        '''
        self.node_name = 'state_grapher'
        print 'Initializing {} connection to ROS master running on {}...'.format(
                self.node_name, self.hostname)
        self.ros = roslibpy.Ros(host=self.hostname, port=9090)
        
        # To be able to plot state estimates:
        self.state_sub = roslibpy.Topic(ros=self.ros, name='/pidrone/state', message_type='pidrone_pkg/State')
        
        # To be able to plot raw sensor data:
        self.ir_sub = roslibpy.Topic(ros=self.ros, name='/pidrone/infrared', message_type='sensor_msgs/Range') # TODO: infrared or raw_infrared?
        # TODO: Later, add Imu and TwistStamped (optical flow) data support
        
    def start_subscribers(self):
        print 'Starting subscribers...'
        self.state_sub.subscribe(self.state_callback)
        self.ir_sub.subscribe(self.ir_data_callback)
        print 'Subscribers started'
        
    def state_callback(self, data):
        print 'Got a State message'
        self.update_plot_state(data)
        
    def ir_data_callback(self, data):
        #print 'Got a Range message'
        self.update_plot_ir(data)
    
    def setup_plots(self):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.time_range = 5 # seconds to show on the x-axis
        
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
        secs = msg['header']['stamp']['secs']
        nsecs = msg['header']['stamp']['nsecs']
        new_time = secs + nsecs*1e-9
        if self.ir_times:
            # If we already have data, check if we should delete old data
            first_time = self.ir_times[0]
            if (new_time - first_time) > self.time_range:
                del self.ir_times[0]
                del self.ir_data[0]
        self.ir_times.append(new_time)
        z_pos = msg['range']
        self.ir_data.append(z_pos)
        print new_time
        
        if len(self.ir_times) == 1:
            x_max = self.ir_times[0] + self.time_range
            do_plot = True
        else:
            x_max = new_time
            # Don't update the plot that often, as it takes a fair amount of
            # time to do so
            if (new_time - self.last_plot_time) > 0.5:
                do_plot = True
            else:
                do_plot = False
                
        if do_plot:
            self.last_plot_time = new_time
            self.update_plot(self.ir_times[0], x_max)
        
    def update_plot(self, x_min, x_max):
        print 'Updating plot'
        # Plot the data
        self.ax.plot(self.ir_times, self.ir_data, color='red')
        self.ax.plot(self.state_times, self.state_data, color='blue')
        self.ax.set_xlim(x_min, x_max)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        #self.fig.canvas.draw_idle()
        
    def run(self):
        self.ros.on_ready(self.start_subscribers, run_in_thread=True)
        self.ros.run_forever()
        
        
def main():
    sg = StateGrapher(hostname='tdrone-blue')
    try:
        # Wait until node is halted
        #rospy.spin()
        sg.run()
    finally:
        # Upon termination of this script, print out a helpful message
        print '{} node terminating.'.format(sg.node_name)
        
if __name__ == '__main__':
    main()
        
    
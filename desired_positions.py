#!/usr/bin/env python2

"""
AER 1217 - Lab 1
Tausif Sharif, 2017
"""

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator

from std_msgs.msg import String

class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""
    # write code here for desired position trajectory generator
    def __init__(self):
    	"""Initialize the ROSDesiredPositionGenerator class."""

    	# Publisher
    	self.pub_des_pos = rospy.Publisher('des_pos', String, queue_size=10)
    	
        # Run the onboard controller at 200 Hz.
    	self.onboard_loop_frequency = 200.
        
        # Run this ROS node at the onboard loop frequency.
        self.run_pub_des_pos = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.send_des_pos)

        #---------------------------------------------------------------------
        # Trajectory is set here.
        #
        # Each row is a point on the trajectory. Add more if needed.
    	# Each row follows the following format: 
    	# 	[desired_x, desired_y. desired_z, desired_yaw]
        #---------------------------------------------------------------------
        """
        # Used to test multiple points for the overall trajectory.
        self.desired_trajectory = np.array([[0, 0, 0.5, 0], 
        									[0, 0, 1, 0],
        									[0.5, 0.5, 1.5, 0], 
        									[1, 1, 1.5, 0], 
        									[1.5, 1.5, 1.5, 0], 
        									[2, 2, 2, 0], 
        									[1.5, 1.5, 1.5, 0], 
        									[1, 1, 1.5, 0],
        									[0.5, 0.5, 1, 0],
        									[0, 0, 1, 0]])
        """

        # Trajectory required by lab.
        self.desired_trajectory = np.array([[0, 0, 1, 0], 
        									[2, 2, 2, 0], 
        									[0, 0, 1, 0]])

        # Finding size of array for debugging purposes.
        self.M, self.N = self.desired_trajectory.shape


    def send_des_pos(self, event):
        """Publish the entire trajectory as a 1D string using the existing String msg type."""
        
        # Creating a 1D flattened array of desired_trajectory.
    	msg_array = self.desired_trajectory.flatten()

        # Converting the 1D msg array to a string and getting rid of brackets. 
    	self.msg_string = ' '.join(map(str, msg_array))

        # Publishing the desired trajectory msg to des_pos.
    	self.pub_des_pos.publish(self.msg_string)
    	

if __name__ == '__main__':
    """Initializing the desired_positions node and starting an instance of ROSDesiredPositionGenerator."""

    rospy.init_node('desired_positions')
    ROSDesiredPositionGenerator()
    rospy.spin()


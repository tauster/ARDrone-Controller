#!/usr/bin/env python2

"""
AER 1217 - Lab 1
Tausif Sharif, 2017
"""

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist

from std_msgs.msg import String
import math

class PositionController(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    # write code here for position controller
    def __init__(self):
    	"""Initialize the PositionController class."""

    	# Initial current position placeholder.
    	self.current_trans_x = 0
    	self.current_trans_y = 0
    	self.current_trans_z = 0

    	# Initial current quaternion placeholder.
    	self.current_rot_x = 0
    	self.current_rot_y = 0
    	self.current_rot_z = 0
    	self.current_rot_w = 0

    	# Initial desired postion for current goal.
    	self.desired_x = 0
    	self.desired_y = 0
    	self.desired_z = 0
    	self.desired_yaw = 0

    	# Placeholder for old variables for differentiation.
    	self.x_old = 0
    	self.y_old = 0
    	self.z_old = 0
    	self.climb_rate_old = 0

        # Initial pose error placeholder.
        self.desired_x_err = 1
        self.desired_y_err = 1
        self.desired_z_err = 1
        self.desired_yaw_err = 1


    def update_position_controller(self, dt):
    	"""Postion Controller.
        
        Parameters
        ----------
        dt: float
            Change in time (seconds) used for differentiation and intergration 
            purposes.
        
        Returns
        -------
        controlled_data: 1x4 array
            This array stores the 4 computed by the position controller.
            [roll, pitch, yaw_rate, climb_rate]
        """
        
    	g = -9.8							# Acceleration due to gravity
    	damping_rt_x = 0.7					# Damping ratios
    	damping_rt_y = 0.7
    	rise_time_x = 0.01					# Rise Times
    	rise_time_y = 0.01
    	rise_time_z = 3
    	rise_time_yaw = 3

        # Updating to the current (x, y, z) positions.
    	x_now = self.current_trans_x
        y_now = self.current_trans_y
    	z_now = self.current_trans_z


    	#---------------------------------------------------------------------
    	# PD Controller
    	#---------------------------------------------------------------------
    	# Finding current climb velocity and acceleration.
    	climb_rate_now = (z_now - self.z_old) / dt
    	climb_accel = (climb_rate_now - self.climb_rate_old) / dt

        # Storing the new z values to the old one for future iterations.
        self.z_old = z_now
        self.climb_rate_old = climb_rate_now

    	# Finding the current quaternions of the UAV.
        quaternion = np.array([self.current_rot_x,
                              self.current_rot_y,
                              self.current_rot_z,
                              self.current_rot_w])

    	# Converting the quaternions to euler angles.
        euler = euler_from_quaternion(quaternion)
        roll_angle = euler[0]
        pitch_angle = euler[1]
        yaw_angle = euler[2]
    	
        # Finding the current x/y velocities.
        x_rate_now = (x_now - self.x_old) / dt
        y_rate_now = (y_now - self.y_old) / dt

        # Setting desired point velocities to 0.
        desired_x_rate = 0
        desired_y_rate = 0
        desired_z_rate = 0

        # Kp and Kd gains for the PD controller.
        Kp_x = 0.1
        Kd_x = 0.00001

        Kp_y = -0.05
        Kd_y = 0.0025

        Kp_z = 0.5
        Kd_z = 0.05

        # PD Controller for x/y/z control.
        x_control = Kp_x*(self.desired_x - x_now) + Kd_x*(desired_x_rate - x_rate_now)
        y_control = Kp_y*(self.desired_y - y_now) + Kd_y*(desired_y_rate - y_rate_now)
        z_control = Kp_z*(self.desired_z - z_now) + Kd_z*(desired_z_rate - climb_rate_now)


    	#---------------------------------------------------------------------
    	# Commanded roll and pitch angles.
    	#---------------------------------------------------------------------
    	# Finding sine and cosine of current yaw for easier callback.
    	S_yaw = np.sin(yaw_angle)
    	C_yaw = np.cos(yaw_angle)
    	
        # Calculating the commanded roll ratio. In the event of an OverflowError (NaN/Inf), set the ratio to 0.
    	try:
    		roll_command_rt = (x_control*S_yaw - y_control*C_yaw) / (math.pow(x_control, 2) + math.pow(y_control, 2) + math.pow((z_control - g), 2))
    	except OverflowError:
    		roll_command_rt = 0
    	
        # Cap the roll ratio to +-0.5 to avoid extreme rolling.
    	if roll_command_rt >= 1/2:
    		roll_command_rt = 1/2
    	if roll_command_rt <= -1/2:
    		roll_command_rt = -1/2

    	# Calculate the commanded roll angle using the roll ratio.
        roll_command = np.arcsin(roll_command_rt)

    	# Cap the max roll to +-0.1 radians.
        if roll_command >= 0.1:
    		roll_command = 0.1
    	if roll_command <= -0.1:
    		roll_command = -0.1

    	# Calculating the commanded pitch ratio.
        pitch_command_rt = (x_control*C_yaw + y_control*S_yaw) / (z_control - g)

    	# Calculate the commanded pitch angle using the pitch ratio.
    	pitch_command = np.arctan(pitch_command_rt)

    	# Cap the max pitch to +-0.1 radians.
        if pitch_command >= 0.1:
    		pitch_command = 0.1
    	if pitch_command <= -0.1:
    		pitch_command = -0.1


    	#---------------------------------------------------------------------
    	# Commanded climb and yaw rates.
    	#---------------------------------------------------------------------
        # Calculate tau for climb and yaw.
    	tau_z = rise_time_z/2.2
    	tau_yaw = rise_time_yaw/2.2
    	
        # Calculate the commanded climb and yaw rates.
    	climb_rate_command = (1/tau_z)*(self.desired_z - self.current_trans_z)
    	yaw_rate_command = (1/tau_yaw)*(self.desired_yaw - yaw_angle)

    	# Cap the max acceleration to 0.075 m/s^2.
        if climb_rate_command >= 0.075:
    		climb_rate_command = 0.075
		

        #---------------------------------------------------------------------
        # Update pose errors.
        #---------------------------------------------------------------------
		# Updating the desired pose error.
		self.desired_x_err = np.absolute((np.absolute(x_now) - (self.desired_x + 0.5)) / (self.desired_x + 0.5))
        self.desired_y_err = np.absolute((np.absolute(y_now) - (self.desired_y + 0.5)) / (self.desired_y + 0.5))
        self.desired_z_err = np.absolute((z_now - (self.desired_z + 0.001)) / (self.desired_z + 0.001))
        self.desired_yaw_err = np.absolute((yaw_angle - (self.desired_yaw + 0.01)) / (self.desired_yaw + 0.01))

    	
        #---------------------------------------------------------------------
        # Return data required.
        #---------------------------------------------------------------------
        controlled_data = np.array([roll_command, pitch_command, yaw_rate_command, climb_rate_command])
    	return controlled_data


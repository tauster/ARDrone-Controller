#!/usr/bin/env python2

"""
AR.Drone's position controller.

PD controller. Needs slight tuning.

Tausif S., 2017

"""

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

        # Current position.
        self.current_trans_x = 0.
        self.current_trans_y = 0.
        self.current_trans_z = 0.

        # Current quaternion.
        self.current_rot_x = 0.
        self.current_rot_y = 0.
        self.current_rot_z = 0.
        self.current_rot_w = 0.

        self.current_yaw = 0

        # Desired postion for trajectory goal.
        self.desired_x = 0.
        self.desired_y = 0.
        self.desired_z = 0.
        self.desired_yaw = 0.

        # Placeholder for old variable for differentiation.
        self.x_old = 0.
        self.y_old = 0.
        self.z_old = 0.
        self.climb_rate_old = 0.

        self.x_desired_old = 0.
        self.y_desired_old = 0.
        self.z_desired_old = 0.

        self.desired_x_err = 1.
        self.desired_y_err = 1.
        self.desired_z_err = 1.
        self.desired_yaw_err = 1.


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
        
        g = 9.8                            # Acceleration due to gravity
        # damping_rt_x = 0.7                    # Damping ratio
        # damping_rt_y = 0.7
        rise_time_z = 1
        rise_time_yaw = 1.

        x_now = self.current_trans_x
        y_now = self.current_trans_y
        z_now = self.current_trans_z

        #---------------------------------------------------------------------
        # Step 1: Calculating current the mass-normalized thrust.
        #---------------------------------------------------------------------
        
        climb_rate_now = (z_now - self.z_old) / dt
        climb_accel = (climb_rate_now - self.climb_rate_old) / dt

        quaternion = np.array([self.current_rot_x,
                              self.current_rot_y,
                              self.current_rot_z,
                              self.current_rot_w])

        euler = euler_from_quaternion(quaternion)
        roll_angle = euler[0]
        pitch_angle = euler[1]
        yaw_angle = euler[2]

        self.current_yaw = yaw_angle

        # f = (climb_accel + g) / (np.cos(roll_angle) * np.cos(pitch_angle))
        f = (0 + g) / (math.cos(roll_angle) * math.cos(pitch_angle))

        self.z_old = z_now
        self.climb_rate_old = climb_rate_now

        #---------------------------------------------------------------------
        # Step 2: Determining the desired x/y accelerations.
        #---------------------------------------------------------------------
        x_rate_now = (x_now - self.x_old) / dt
        y_rate_now = (y_now - self.y_old) / dt

        # print("x/y rate: {0} {1}".format(x_rate_now, y_rate_now))

        desired_x_rate = (self.desired_x - self.x_desired_old)/dt
        desired_y_rate = (self.desired_y - self.y_desired_old)/dt
        # desired_z_rate = 0

        damping_rt_x = 1
        damping_rt_y = 1
        
        w_n_x = 1
        w_n_y = 1

        x_accel = 2.0*damping_rt_x*w_n_x*(desired_x_rate - x_rate_now) + math.pow(w_n_x, 2)*(self.desired_x - x_now)
        y_accel = 2.0*damping_rt_y*w_n_y*(desired_y_rate - y_rate_now) + math.pow(w_n_y, 2)*(self.desired_y - y_now)

        # print("x/y accel: {0} {1}".format(x_accel, y_accel))
        print("x/y/z des: {0} {1} {2}".format(self.desired_x, self.desired_y, self.desired_z))
        print("x/y/z now: {0} {1} {2}".format(x_now, y_now, z_now))

        self.x_old = x_now
        self.y_old = y_now
        self.x_desired_old = self.desired_x
        self.y_desired_old = self.desired_y

        #---------------------------------------------------------------------
        # Step 3: Calculating commanded roll and pitch.
        #---------------------------------------------------------------------
        roll_command_rt = -y_accel / f

        if roll_command_rt >= 1.:
            roll_command_rt = 1.
        if roll_command_rt <= -1.:
            roll_command_rt = -1.

        roll_command = math.asin(roll_command_rt)
        
        pitch_command_rt = x_accel / (f*math.cos(roll_command))

        if pitch_command_rt >= 1.:
            pitch_command_rt = 1.
        if pitch_command_rt <= -1.:
            pitch_command_rt = -1.
        
        pitch_command = math.asin(pitch_command_rt)

        if yaw_angle >= 0 or yaw_angle <= 0:
            roll_command_B = (roll_command*math.cos(yaw_angle)) + (pitch_command*math.sin(yaw_angle))
            pitch_command_B = (-roll_command*math.sin(yaw_angle)) + (pitch_command*math.cos(yaw_angle))
            roll_command = roll_command_B
            pitch_command = pitch_command_B
        
        #---------------------------------------------------------------------
        # Step 4: Calculating commanded climb and yaw rates.
        #---------------------------------------------------------------------
        tau_z = rise_time_z/2.2
        tau_yaw = rise_time_yaw/2.2
        
        climb_rate_command = (1/tau_z)*(self.desired_z - self.current_trans_z)

        if climb_rate_command >= 2:
            climb_rate_command = 2

        # Ensuring the yaw angles are in the respective quadrant.
        for i in range(0, 1):
            if ((355*(np.pi/180)) <= self.desired_yaw <= (5*(np.pi/180))):
                break
            elif (yaw_angle < 0) and ((6*(np.pi/180)) <= self.desired_yaw <= (354*(np.pi/180))):
                yaw_angle = yaw_angle + (2*np.pi)
                break
            else:
                pass

        yaw_rate_command = (0.85)*(self.desired_yaw - yaw_angle)

        # Updating the desired pose error.
        self.desired_x_err = np.absolute((np.absolute(x_now) - (self.desired_x + 0.5)) / (self.desired_x + 0.5))
        self.desired_y_err = np.absolute((np.absolute(y_now) - (self.desired_y + 0.5)) / (self.desired_y + 0.5))
        self.desired_z_err = np.absolute((z_now - (self.desired_z + 0.001)) / (self.desired_z + 0.001))
        self.desired_yaw_err = np.absolute((yaw_angle - (self.desired_yaw + 0.01)) / (self.desired_yaw + 0.01))
        
        # Returning the commanded roll/pitch/yaw/climb
        controlled_data = np.array([roll_command, pitch_command, yaw_rate_command, climb_rate_command])

        return controlled_data

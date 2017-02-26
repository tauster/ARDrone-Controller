#!/usr/bin/env python2

"""
AER 1217 - Lab 1
Tausif Sharif, 2017
"""

"""ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

import rosbag

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
from position_controller import PositionController

from std_msgs.msg import String

class ROSControllerNode(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    # write code here to define node publishers and subscribers
    # publish to /cmd_vel topic
    # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback
    def __init__(self):
        """Initialize the ROSControllerNode class."""

        # Publishers
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', 
                                            Twist,
                                            queue_size = 300)
        

        # Subscribers
        self.model_name = 'ARDroneCarre'
        
        self.sub_vicon_data = rospy.Subscriber('/vicon/{0}/{0}'.format(
                                              self.model_name),
                                              TransformStamped, self.update_vicon_data)

        self.sub_des_pos = rospy.Subscriber('des_pos', String, self.update_desired_position)

        # Initialize messages for publishing.
        self.cmd_vel_msg = Twist()

        # Run the onboard controller at 200 Hz.
        self.onboard_loop_frequency = 200.
        
        # Initializing an instance of the position controller to pass/calculate data.
        self.pos_class = PositionController()

        # Run this ROS node at the onboard loop frequency.
        self.run_pub_cmd_vel = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.update_roll_pitch_yaw)

        # Placeholder to keep track of time for trajectory planning and differentiation.
        self.current_point = 1
        self.nonunix_time = 0
        self.dt = 0

        # Keep track of time for differentiation and integration within the controller.
        self.old_time = rospy.get_time()


    def update_roll_pitch_yaw(self, event):
        """Determine the roll/pitch angles, yaw/climb rates and publish these values."""
        
        # Determine the time step for differentiation.
        current_time = rospy.get_time()
        self.dt = current_time - self.old_time
        
        # Get the next set of roll/pitch/yaw/climb commands from the position controller.
        new_controls = self.pos_class.update_position_controller(self.dt)

        # Rearranging the data for ease.
        [new_roll, new_pitch, new_yaw_rate, new_climb_rate] = new_controls
        
        # Setting up the new controls in the cmd_vel msg format.
        self.cmd_vel_msg.linear.x = new_roll
        self.cmd_vel_msg.linear.y = new_pitch
        self.cmd_vel_msg.angular.z = new_yaw_rate
        self.cmd_vel_msg.linear.z = new_climb_rate
        
        # Publish the new controls to the cmd_vel msg.
        self.pub_cmd_vel.publish(self.cmd_vel_msg)
        
        # Set the old time to the current for future time step calculations.
        self.old_time = current_time


    def update_vicon_data(self, vicon_data_msg):
        """Get the current gazebo position and attitude for the position controller."""
        
        # Update the quadrotor states as an instance of pos_class (postion controller).
        (self.pos_class.current_trans_x,
        self.pos_class.current_trans_y,
        self.pos_class.current_trans_z) = (vicon_data_msg.transform.translation.x,
             vicon_data_msg.transform.translation.y,
             vicon_data_msg.transform.translation.z)
        
        (self.pos_class.current_rot_x,
        self.pos_class.current_rot_y,
        self.pos_class.current_rot_z,
        self.pos_class.current_rot_w) = (vicon_data_msg.transform.rotation.x,
             vicon_data_msg.transform.rotation.y,
             vicon_data_msg.transform.rotation.z,
             vicon_data_msg.transform.rotation.w)


    def update_desired_position(self, pos_msg):
        """Acquires the entire trajectory plan from the des_pos msg and controls the desired postions."""
        
        # Acquires the des_pos msg as a string and converts it to a 1D array.
        self.des_pos_msg = np.fromstring(pos_msg.data, dtype = float, sep = ' ')

        # Finds the number of points in the trajectory.
        num_points = self.des_pos_msg.size/4

        # Reshapes the 1D msg array to a (num_points)x4 matrix where each row is a desired pose (x, y, z, yaw).
        trajectory = np.reshape(self.des_pos_msg, (-1, num_points + 1))

        # Sets the "current" desired pose for the pos_class instance.
        self.pos_class.desired_x = trajectory[self.current_point - 1, 0]
        self.pos_class.desired_y = trajectory[self.current_point - 1, 1]
        self.pos_class.desired_z = trajectory[self.current_point - 1, 2]
        self.pos_class.desired_yaw = trajectory[self.current_point - 1, 3]

        # Finding nonunix time to keep track of how many seconds each desired pose can get.
        self.nonunix_time += self.dt

        # Each "current" pose has 30 seconds to hold its position with 20% error
        # Once time is up, current_point is incremented to allow for the next point in the trajectory.
        # Nonunix time is reset. 
        if self.pos_class.desired_x_err <= 20 and self.pos_class.desired_y_err <= 20 and self.pos_class.desired_z_err <= 20:
            if self.current_point < num_points and self.nonunix_time >= 30:
                self.current_point += 1
                self.nonunix_time = 0


if __name__ == '__main__':
    """Initializing the ros_controller node and starting an instance of ROSControllerNode."""

    rospy.init_node('ros_controller')
    ROSControllerNode()
    rospy.spin()


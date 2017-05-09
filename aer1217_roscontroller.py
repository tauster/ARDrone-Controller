#!/usr/bin/env python2

"""
ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel_RHC

Tausif S., 2017

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
        """
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', 
                                            Twist,
                                            queue_size = 300)
        """

        self.pub_cmd_vel = rospy.Publisher('cmd_vel_RHC', 
                                            Twist,
                                            queue_size = 300)
        
        self.pubTakePic  = rospy.Publisher('/ardrone/take_pic', String, queue_size=10)

        # Subscribers
        self.model_name = 'ARDroneCarre'
        
        self.sub_vicon_data = rospy.Subscriber('/vicon/{0}/{0}'.format(
                                              self.model_name),
                                              TransformStamped, self.update_vicon_data)

        self.sub_des_pos = rospy.Subscriber('des_pos', String, self.update_desired_position)

        # Initialize messages for publishing
        self.cmd_vel_msg = Twist()

        # Run the onboard controller at 200 Hz
        self.onboard_loop_frequency = 200.
        
        # Calling the position controller to pass the data
        self.pos_class = PositionController()

        # Run this ROS node at the onboard loop frequency
        self.run_pub_cmd_vel = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.update_roll_pitch_yaw)

        self.current_point = 1
        self.nonunix_time = 0
        self.dt = 0

        self.num_points_old = 3

        self.pic_number = 1

        # Keep time for differentiation and integration within the controller
        self.old_time = rospy.get_time()
        self.start_time = rospy.get_time()


    def update_roll_pitch_yaw(self, event):
        """Determine the motor speeds and and publishes these."""
        
        # Determine the time step for differentiation and integration
        current_time = rospy.get_time()
        self.dt = current_time - self.old_time
        
        # Get the next set of postioning commands from the position controller
        new_controls = self.pos_class.update_position_controller(self.dt)

        # Rearranging the data for ease
        [new_roll, new_pitch, new_yaw_rate, new_climb_rate] = new_controls
        
        # Setting the cmd_vel msg values to the desired ones
        self.cmd_vel_msg.linear.x = new_roll
        self.cmd_vel_msg.linear.y = new_pitch
        self.cmd_vel_msg.angular.z = new_yaw_rate
        self.cmd_vel_msg.linear.z = new_climb_rate
        

        # Publish the motor commands for the ardrone plugin
        self.pub_cmd_vel.publish(self.cmd_vel_msg)
        
        # Set the old time to the current for future time step calculations
        self.old_time = current_time


    def update_vicon_data(self, vicon_data_msg):
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
        self.des_pos_msg = np.fromstring(pos_msg.data, dtype = float, sep = ' ')

        num_points = self.des_pos_msg.size/4
        if num_points != self.num_points_old:
            self.num_points_old = num_points
            self.current_point = 1

        trajectory = np.reshape(self.des_pos_msg, (-1, 4))

        self.pos_class.desired_x = trajectory[self.current_point - 1, 0]
        self.pos_class.desired_y = trajectory[self.current_point - 1, 1]
        self.pos_class.desired_z = trajectory[self.current_point - 1, 2]
        self.pos_class.desired_yaw = trajectory[self.current_point - 1, 3]

        self.nonunix_time += self.dt

        #---------------------------------------------------
        # SCANNING PHASE.
        #---------------------------------------------------
        self.pubTakePic.publish("no")
        #---------------------------------------------------

        if (((self.pos_class.desired_x - 0.15) < self.pos_class.current_trans_x < (self.pos_class.desired_x + 0.15)) and
            ((self.pos_class.desired_y - 0.15) < self.pos_class.current_trans_y < (self.pos_class.desired_y + 0.15)) and
            ((self.pos_class.desired_z - 0.05) < self.pos_class.current_trans_z < (self.pos_class.desired_z + 0.05)) and
            ((self.pos_class.desired_yaw - 0.25) < self.pos_class.current_yaw < (self.pos_class.desired_yaw + 0.25))):
            if self.current_point < num_points and self.nonunix_time >= 0.01:
                self.current_point += 1
                self.nonunix_time = 0

                #---------------------------------------------------
                # SCANNING PHASE.
                #---------------------------------------------------
                
                current_mission_time = int(self.start_time - (rospy.get_time()))
                file_name = str(round(self.pic_number)) + "_t" + str(current_mission_time) + "_X" + str(round(self.pos_class.current_trans_x, 2)) + "_Y" + str(round(self.pos_class.current_trans_y, 2)) + "_Z" + str(round(self.pos_class.current_trans_z, 2)) + "_W" + str(round(self.pos_class.current_yaw, 2)) + ".jpg"
                self.pic_number = self.pic_number + 1
                self.pubTakePic.publish(file_name)
                
                #---------------------------------------------------



if __name__ == '__main__':
    rospy.init_node('ros_controller')
    ROSControllerNode()
    rospy.spin()


#!/usr/bin/env python2

"""
ROS Node for publishing desired positions.

Waits for keyboard commands related to different trajectories.
Publishes entire trajectory as a single flat string.

Tausif S., 2017

"""

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

        # Sub
        self.sub_cen = rospy.Subscriber('/ardrone/center', String, self.ReceiveCenter)
        self.sub_linear = rospy.Subscriber('/ardrone/linear', String, self.ReceiveLinear)
        self.sub_circle = rospy.Subscriber('/ardrone/circle', String, self.ReceiveCircle)
        self.sub_ntraj = rospy.Subscriber('/ardrone/ntraj', String, self.ReceiveNTrajectory)

        self.sub_ntraj = rospy.Subscriber('/ardrone/racetraj', String, self.ReceiveRace)

        self.sub_defaultH = rospy.Subscriber('/ardrone/default_h', String, self.ReceiveDefaultH)
        self.sub_defaultH = rospy.Subscriber('/ardrone/emergency_h', String, self.ReceiveEmergencyH)

        self.sub_HoopHeight = rospy.Subscriber('/ardrone/hoop_height', String, self.UpdateHoopHeight)

        # Publisher
        self.pub_des_pos = rospy.Publisher('des_pos', String, queue_size=10)
        
        # Run the onboard controller at 200 Hz.
        self.onboard_loop_frequency = 200.

        self.current_point = 1
        
        # Run this ROS node at the onboard loop frequency.
        self.run_pub_des_pos = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.send_des_pos)

        #---------------------------------------------------------------------
        # Trajectory is set here.
        #
        # Each row is a point on the trajectory. Add more if needed.
        # Each row follows the following format: 
        #   [desired_x, desired_y. desired_z, desired_yaw]
        #---------------------------------------------------------------------
        
        # Default trajectory, race starting point.
        self.desired_trajectory = np.array([[1.37, 2.3, 2.4, 0], 
                                            [1.37, 2.3, 2.4, 0], 
                                            [1.37, 2.3, 2.4, 0]])

        #---------------------------------------------------------------------
        # Default center position.
        #---------------------------------------------------------------------
        self.center_position = self.desired_trajectory

        #---------------------------------------------------------------------
        # "Race" trajectory setup.
        #---------------------------------------------------------------------
        self.race_hieght = 1.3
        # self.race_hieght = 2.2
        self.race_trajectory = np.array([[1.37, 2.3, self.race_hieght, round((279*(np.pi/180)), 2)], 
                                         [1.45, 1.84, self.race_hieght, round((270*(np.pi/180)), 2)], 
                                         [1.45, 1.58, self.race_hieght, round((278*(np.pi/180)), 2)],
                                         [1.66, 0.16, self.race_hieght, round((196*(np.pi/180)), 2)],
                                         [1.44, 0.09, self.race_hieght, round((276*(np.pi/180)), 2)],
                                         [1.5, -0.5, self.race_hieght, round((270*(np.pi/180)), 2)],
                                         [1.5, -1, self.race_hieght, round((197*(np.pi/180)), 2)],
                                         [0.85, -1.2, self.race_hieght, round((200*(np.pi/180)), 2)],
                                         [0.52, -1.33, self.race_hieght, round((178*(np.pi/180)), 2)],
                                         [0.01, -1.31, self.race_hieght, round((163*(np.pi/180)), 2)],
                                         [-0.41, -1.18, self.race_hieght, round((129*(np.pi/180)), 2)],
                                         [-0.68, -0.86, self.race_hieght, round((65*(np.pi/180)), 2)],
                                         [-0.52, -0.51, self.race_hieght, round((103*(np.pi/180)), 2)],
                                         [-0.61, -0.14, self.race_hieght, round((115*(np.pi/180)), 2)],
                                         [-0.73, 0.12, self.race_hieght, round((186*(np.pi/180)), 2)],
                                         [-0.94, 0.09, self.race_hieght, round((164*(np.pi/180)), 2)],
                                         [-1.18, 0.16, self.race_hieght, round((150*(np.pi/180)), 2)],
                                         [-1.37, 0.27, self.race_hieght, round((92*(np.pi/180)), 2)],
                                         [-1.4, 0.88, self.race_hieght, round((93*(np.pi/180)), 2)],
                                         [-1.44, 1.56, self.race_hieght, round((90*(np.pi/180)), 2)],
                                         [-1.44, 2.25, self.race_hieght, round((357*(np.pi/180)), 2)],
                                         [-0.47, 2.2, self.race_hieght, round((3*(np.pi/180)), 2)],
                                         [0.07, 2.25, self.race_hieght, round((0*(np.pi/180)), 2)],
                                         [0.31, 2.25, self.race_hieght, round((313*(np.pi/180)), 2)],
                                         [0.63, 1.9, self.race_hieght, round((250*(np.pi/180)), 2)],
                                         [0.44, 1.35, self.race_hieght, round((309*(np.pi/180)), 2)],
                                         [0.63, 1.11, self.race_hieght, round((307*(np.pi/180)), 2)],
                                         [1.1, 0.5, self.race_hieght, round((264*(np.pi/180)), 2)],
                                         [1.06, 0.15, self.race_hieght, round((189*(np.pi/180)), 2)],
                                         [0.74, 0.1, self.race_hieght, round((178*(np.pi/180)), 2)],
                                         [0.43, 0.11, self.race_hieght, round((158*(np.pi/180)), 2)],
                                         [0.15, 0.22, self.race_hieght, round((130*(np.pi/180)), 2)],
                                         [-0.25, 0.68, self.race_hieght, round((137*(np.pi/180)), 2)],
                                         [-0.6, 1, self.race_hieght, round((141*(np.pi/180)), 2)],
                                         [-1.48, 1.71, self.race_hieght, round((143*(np.pi/180)), 2)],
                                         [-2.5, 2.47, self.race_hieght, round((143*(np.pi/180)), 2)],])

        # self.race_trajectory[:, 3] = 0

        #---------------------------------------------------------------------
        # Linear trajectory setup.
        #---------------------------------------------------------------------
        linear_start = np.array([[0, 0, 1.3, 0]])
        linear_end = np.array([[-2, 2, 1.3, 0]])

        # Number of intermediate points (one way).
        linear_intpoints = 6

        # Placeholder for intermediate points.
        linear_midpoints = np.ones((linear_intpoints, 4))

        # Calculate each intermediate point required (one way).
        for i in range(1, linear_intpoints+1):
            linear_midpoints[i-1,:] = linear_start + (i/(linear_intpoints+1))*((linear_end) - (linear_start))

        # Concatenating the trajectory together. 
        # Flipping the intermediate points for reverse direction.
        """
        self.linear_trajectory = np.concatenate((linear_start, 
                                            linear_midpoints, 
                                            linear_end, 
                                            np.flipud(linear_midpoints), 
                                            linear_start), axis = 0)
        """
        # Non flip (linear trajectory one-way).
        self.linear_trajectory = np.concatenate((linear_start, linear_midpoints, linear_end), axis = 0)

        #---------------------------------------------------------------------
        # Setting up circular trajectory.
        #---------------------------------------------------------------------
        circular_origin = np.array([[0, 1, 1.5, 0]])
        circular_radius = 1

        # Number of circular waypoints points.
        circular_waypoints = 12

        # Placeholder for circular trajectory points.
        self.circular_trajectory = np.ones((circular_waypoints, 4))

        # Calculate each circular trajectory point required (one cycle).
        for i in range(1, circular_waypoints+1):
            angle_segment = i*((2*np.pi)/circular_waypoints)
            self.circular_trajectory[i-1,:] = [circular_origin[0,0] + circular_radius*np.cos(angle_segment), 
                                        circular_origin[0,1] + circular_radius*np.sin(angle_segment), 
                                        circular_origin[0,2],
                                        0]

        #---------------------------------------------------------------------
        # Setting up N trajectory.
        #---------------------------------------------------------------------
        #---------------------------------------------------
        # Environment info
        #---------------------------------------------------
        # Desired altitude.
        desired_alt = 1.75

        # Working area coordinates.
        working_area_x = np.array([-1.5, 1])
        working_area_y = np.array([-1, 2])

        # Number of subdivided rows and columns (number of pictures = num_rows*run_cols)
        num_rows = 4
        num_cols = 5


        #---------------------------------------------------
        # Setting up initial subdivision centroid
        #---------------------------------------------------
        # The overall x/y absolute lengths.
        x_length = np.absolute(working_area_x[1] - working_area_x[0])
        y_length = np.absolute(working_area_y[1] - working_area_y[0])

        # X/y lengths of subdivisions.
        sub_x = round((x_length/num_cols), 1)
        sub_y = round((y_length/num_rows), 1)

        # Using the max of the subdivision rectangles to idealize square subdivisions.
        ideal_sq = np.amax([sub_x, sub_y])

        # Finding absolute centroid of the idealized square.
        centroid_i = ideal_sq/2


        #---------------------------------------------------
        # Setting up initial subdivision centroid
        #---------------------------------------------------
        # The first point of the sequence of subdivisions (bottom left subdivision centroid).
        current_point = np.array([working_area_x[0] + centroid_i, working_area_y[0] + centroid_i])

        # Setting up one x/row sequence (change in columns).  
        row_sequence = np.zeros((num_cols, 4))

        # One way row sequence (left to right).
        for i in range(0, num_cols):
            row_sequence[i, 0] = round(current_point[0], 2)
            current_point[0] = current_point[0] + ideal_sq

        # Adding the y values of the first row.
        row_sequence[:, 1] = current_point[1]

        # Matrix of all centroid trajectory points.
        self.n_trajectory = np.zeros((num_cols*num_rows, 4))

        # Adding each row sequence for every row needed.
        for i in range(0, num_rows):
            # Making sure the x/column values travel in an n-pattern by flipping after each row.
            if (i % 2) == 0:
                self.n_trajectory[((i*num_rows)+i):((i*num_cols)+num_cols), :] = row_sequence
            else:
                self.n_trajectory[((i*num_rows)+i):((i*num_cols)+num_cols), :] = np.flipud(row_sequence)

            # Adding the y values of the current row and incrementing for next row.
            self.n_trajectory[((i*num_rows)+i):((i*num_cols)+num_cols), 1] = current_point[1]
            current_point[1] = current_point[1] + ideal_sq

        self.n_trajectory[:, 2] = desired_alt

        #---------------------------------------------------
        # Setting up hoop height change
        #---------------------------------------------------
        self.hoopH = 1.3
        

    #---------------------------------------------------------------------
    # Set the desired trajectory for the relevant key pressed.
    #---------------------------------------------------------------------
    def ReceiveDefaultH(self, defaulth_msg):
        # Sets race trajectory's height to 1.3 when I is pressed.
        if defaulth_msg.data == "I":
            self.race_trajectory[:, 2] = 1.3

    def ReceiveEmergencyH(self, emergency_msg):
        # Sets race trajectory's height to 1.3 when O is pressed.
        if emergency_msg.data == "O":
            self.race_trajectory[:, 2] = 2.4

    def ReceiveCenter(self, cen_msg):
        # Sets trajectory to center position (0, 0, 1) when V is pressed.
        if cen_msg.data == "V":
            self.desired_trajectory = self.center_position

            # Finding size of array for debugging purposes.
            self.M, self.N = self.desired_trajectory.shape

    def ReceiveRace(self, race_msg):
        # Sets trajectory to race trajectory when C is pressed.
        if race_msg.data == "C":
            self.desired_trajectory = self.race_trajectory

            # Finding size of array for debugging purposes.
            self.M, self.N = self.desired_trajectory.shape

    def ReceiveLinear(self, lin_msg):
        # Sets trajectory to linear trajectory when N is pressed.
        if lin_msg.data == "N":
            self.desired_trajectory = self.linear_trajectory

            # Finding size of array for debugging purposes.
            self.M, self.N = self.desired_trajectory.shape

    def ReceiveCircle(self, cir_msg):
        # Sets trajectory to circular trajectory when M is pressed.
        if cir_msg.data == "M":
            self.desired_trajectory = self.circular_trajectory

            # Finding size of array for debugging purposes.
            self.M, self.N = self.desired_trajectory.shape

    def ReceiveNTrajectory(self, ntraj_msg):
        # Sets trajectory to scanning N-trajectory when B is pressed.
        if ntraj_msg.data == "B":
            self.desired_trajectory = self.n_trajectory

            # Finding size of array for debugging purposes.
            self.M, self.N = self.desired_trajectory.shape

    def UpdateHoopHeight(self, hoop_msg):
        # Updates hoop height to the race trajectory when detected.
        if hoop_msg.data != "no":
            self.hoopH = round(float(hoop_msg.data), 2)
            self.race_hieght = self.hoopH

            self.race_trajectory[:, 2] = self.hoopH


    #---------------------------------------------------------------------

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

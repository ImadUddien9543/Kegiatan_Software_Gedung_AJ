#!/usr/bin/env python

import rospy
from master_krai.msg import Wheel
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import numpy as np
import math

class odometry():
	sudut = rospy.get_param("/Kinematik/sudut_omni")
	matrix_A = np.zeros(shape=(4, 3))
	inv_matrix_A = np.zeros(shape=(3, 4))	
	rot_matrix = np.zeros(shape=(3, 3))		
	omni_rpm = np.zeros(shape=(4, 1))
	omni_dist = np.zeros(shape=(4, 1))
	pos_matrix = np.zeros(shape=(3, 1))
	vel_matrix = np.zeros(shape=(3, 1))

	wheel_value = Wheel()
	prev_joy_msg = Joy()

	x_axis : float = 0
	y_axis : float = 0
	z_axis : float = 0
	reset_read : bool = False
	
	def __init__(self):
		self.omni_rpm_sub = rospy.Subscriber('/Gajah/rpm_enc', Wheel, self.func_cb)
		self.PS4_joy_sub = rospy.Subscriber('/joy', Joy, self.Command_CB)
		self.reset_state = rospy.Publisher('/Gajah/reset_reading', Bool, queue_size = 1)	

	def Command_CB(self, data):
		if msg.buttons != self.prev_joy_msg.buttons:
			self.prev_joy_msg = msg
			if msg.buttons[2]: #R3_B
				self.reset_read = not self.reset_read

		self.reset_state.publish(self.reset_read)

	def func_cb(self, data):
		self.omni_rpm[0] = data.RF_rpm
		self.omni_rpm[1] = data.LF_rpm
		self.omni_rpm[2] = data.LB_rpm		
		self.omni_rpm[3] = data.RB_rpm

		self.omni_dist[0] = data.RF_dist
		self.omni_dist[1] = data.LF_dist
		self.omni_dist[2] = data.LB_dist
		self.omni_dist[3] = data.RB_dist

		self.fwd_kinematics()

	def fwd_kinematics(self):
		self.matrix_A[0, 0] = -math.sin(math.radians(self.sudut[0] + self.theta))
		self.matrix_A[1, 0] = -math.sin(math.radians(self.sudut[1] + self.theta))
		self.matrix_A[2, 0] = -math.sin(math.radians(self.sudut[2] + self.theta))
		self.matrix_A[3, 0] = -math.sin(math.radians(self.sudut[3] + self.theta))

		self.matrix_A[0, 1] = math.cos(math.radians(self.sudut[0] + self.theta))
		self.matrix_A[1, 1] = math.cos(math.radians(self.sudut[1] + self.theta))
		self.matrix_A[2, 1] = math.cos(math.radians(self.sudut[2] + self.theta))
		self.matrix_A[3, 1] = math.cos(math.radians(self.sudut[3] + self.theta))
		self.matrix_A[0, 2] = self.matrix_A[1, 2] = self.matrix_A[2, 2] = self.matrix_A[3, 2] = self.R_robot
		
		self.rot_matrix[0, 0] = math.cos(math.radians(self.theta))
		self.rot_matrix[1, 0] = math.sin(math.radians(self.theta))		
		self.rot_matrix[0, 1] = -math.sin(math.radians(self.theta))
		self.rot_matrix[1, 1] = math.cos(math.radians(self.theta))
		self.rot_matrix[0, 2] = self.rot_matrix[1, 2] = self.rot_matrix[2, 0] = self.rot_matrix[2, 1] = 0
		self.rot_matrix[2, 2] = 1

		self.inv_matrix_A = pinv(self.matrix_A)
		self.vel_matrix = np.matmul(self.inv_matrix_A, self.omni_rpm)
		self.pos_matrix = np.matmul(self.rot_matrix, self.vel_matrix)

	def estimate_positon(self):
		self.x_axis = (1/2) * (self.omni_rpm[])

if __name__ == '__main__':
	try:
		rospy.init_node('odometry', anonymous = True)
		rospy.logwarn("odometry node is running")
		odom = odometry()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import JoyFeedback
from sensor_msgs.msg import Joy
from master_krai.msg import Wheel
import numpy as np
import math
import yaml

class kinematics():	
	alpha_coeff = rospy.get_param("/Kinematik/alpha_coeff")
	vel_coeff = rospy.get_param("/Kinematik/vel_coeff")
	sudut = rospy.get_param("/Kinematik/sudut_omni")
	R_robot = rospy.get_param("/Kinematik/r_robot")
	poly_var = rospy.get_param("/Kinematik/poly_quintic")
	omni_val = rospy.get_param("/Kinematik/omni_val")
	omni_pwm = rospy.get_param("/Kinematik/omni_pwm")
	nos_plus = rospy.get_param("/Kinematik/nos_plus")
	nos_min = rospy.get_param("/Kinematik/nos_min")	
	s_quintic = np.poly1d(poly_var)
	v_quintic = np.polyder(s_quintic)

	matrix_A = np.zeros(shape=(4, 3))
	matrix_B = np.zeros(shape=(3, 1))
	matrix_C = np.zeros(shape=(3, 3))
	matrix_D = np.zeros(shape=(3, 1))	
	result_matrix = np.zeros(shape=(4, 1))	
	
	#msg
	#comment unused message! bisa bikin lag rosserial(?)
	wheel_value = Wheel()	
	prev_joy_msg = Joy()		

	#variables			
	theta : float = 0.0
	omega : float = 0.0
	vel : float = 0.0
	vel_x : float = 0.0
	vel_y : float = 0.0	
	avel_l : float = 0.0
	avel_r : float = 0.0
	nos : float = 0
	nos_plus : float = 1.0	
	lock : bool = False

	loop_rate_ = 200
	acc_lim = 500
	v = vel_x
	oldv = 0

	def __init__(self):
		self.PS4_joy_sub = rospy.Subscriber('/joy', Joy, self.Command_CB)				
		self.STM32_pwm = rospy.Publisher('/Gajah/Four_OWD_Kinematics', Wheel, queue_size = 1)		

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
		
		self.matrix_C[0, 0] = math.cos(math.radians(self.theta))
		self.matrix_C[1, 0] = math.sin(math.radians(self.theta))		
		self.matrix_C[0, 1] = -math.sin(math.radians(self.theta))
		self.matrix_C[1, 1] = math.cos(math.radians(self.theta))
		self.matrix_C[0, 2] = self.matrix_C[1, 2] = self.matrix_C[2, 0] = self.matrix_C[2, 1] = 0
		self.matrix_C[2, 2] = 1

		self.matrix_B[0] = self.vel_x * self.vel_coeff
		self.matrix_B[1] = self.vel_y * self.vel_coeff
		self.matrix_B[2] = self.omega * self.alpha_coeff

		self.result_matrix = np.matmul(self.matrix_A, np.matmul(self.matrix_C, self.matrix_B))
		self.result_matrix = np.clip(np.dot(self.result_matrix, self.nos), self.omni_val[0], self.omni_val[1])
		self.result_matrix = np.interp(self.result_matrix, self.omni_val, self.omni_pwm)
		
		self.wheel_value.R_Front = math.ceil(np.rint(self.result_matrix[0]))
		self.wheel_value.L_Front = math.ceil(np.rint(self.result_matrix[1]))
		self.wheel_value.L_Back = math.ceil(np.rint(self.result_matrix[2]))
		self.wheel_value.R_Back = math.ceil(np.rint(self.result_matrix[3]))
							
		self.STM32_pwm.publish(self.wheel_value)


	def Command_CB(self, msg):															
		self.vel_x = -msg.axes[0]
		self.vel_y = -msg.axes[1]
		self.theta = math.degrees(math.atan2(self.vel_y, self.vel_x))
		if self.theta < 0.0:	self.theta += 360.0

		if msg.buttons[2] == 1:	self.nos_plus += 0.02 #segitiga
		elif msg.buttons[2] == 0:	self.nos_plus -= 0.85

		if msg.buttons[6]== 1:		self.avel_l = -msg.axes[4] #L2
		elif msg.buttons[6] == 0:	self.avel_l = 0.0

		if msg.buttons[7] == 1:		self.avel_r = msg.axes[5] #R2
		elif msg.buttons[7] == 0: 	self.avel_r = 0.0			

		self.omega = self.avel_r + self.avel_l
		self.nos_plus = np.clip(self.nos_plus, 4, 9)
		self.nos = np.clip(self.v_quintic(self.nos_plus), 18, 100)

if __name__ == '__main__':
	try:
		rospy.init_node('Kinematik', anonymous = True)
		rospy.logwarn("Kinematik node is running")
		gajah = kinematics()
		rate = rospy.Rate(200)  # 10hz
		while not rospy.is_shutdown():
			gajah.fwd_kinematics()
			rate.sleep()
	
	except rospy.ROSInterruptException:
		rospy.logwarn("Kinematik program interrupted before completion")

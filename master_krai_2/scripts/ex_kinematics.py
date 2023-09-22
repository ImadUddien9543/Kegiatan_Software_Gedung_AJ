#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from master_krai.msg import Wheel
from master_krai.math_func import *
from math import atan2
from numpy import interp, rint, clip, pi #sebisa mungkin lgsg import fungsi dr module yg dipake, utk mempercepat python

class kinematics():	
	ang_coeff = rospy.get_param("/kinematik/ang_coeff")
	vel_coeff = rospy.get_param("/kinematik/vel_coeff")
	alpha = radians(rospy.get_param("/kinematik/roda4_wheel_alpha"))
	gamma = radians(rospy.get_param("/kinematik/roda4_wheel_gamma"))
	r = rospy.get_param("/kinematik/r_omni")
	
	poly_var = rospy.get_param("/kinematik/poly_quintic")
	omni_val = rospy.get_param("/kinematik/omni_val")
	omni_pwm = rospy.get_param("/kinematik/omni_pwm")
	s_quintic = np.poly1d(poly_var)
	v_quintic = np.polyder(s_quintic)

	#msg
	o = Wheel()	
	prev_joy = Joy()		

	#variables			
	theta : float = 0.0	
	vel_x : float = 0.0
	vel_y : float = 0.0
	vel_z : float = 0.0	
	avel_l : float = 0.0
	avel_r : float = 0.0
	nos : float = 0
	nos_plus : float = 1.0

	def __init__(self):
		self.PS4_joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)				
		self.STM32_pwm = rospy.Publisher('/robot/kinematics', Wheel, queue_size=10)		
		self.timer_pub = rospy.Timer(rospy.Duration(1.0 / 50.0), self.vel_pub) #50Hz

	def vel_pub(self):
		rot_inv = inv_rot(self.theta)
		j_4 = jacobian_omni4(self.alpha, self.gamma, self.r)
		vel = vel_vector(self.vel_x, self.vel_y, self.vel_z)

		result = j_4 * rot_inv * vel
		result = interp_(result, self.omni_val, self.omni_pwm)

		self.o.R_Front = result[0]
		self.o.L_Front = result[1]
		self.o.L_Back = result[2]
		self.o.R_Back = result[3]
		self.STM32_pwm.publish(self.wheel_value)


	def joy_cb(self, msg):															
		self.vel_x = msg.axes[0] * self.vel_coeff
		self.vel_y = msg.axes[1] * self.vel_coeff
		self.theta = atan2(self.vel_y, self.vel_x) #radian
		if self.theta < 0.0:	self.theta += (2*pi)

		if msg.buttons[2] == 1:		self.nos_plus += 0.02 #segitiga
		elif msg.buttons[2] == 0:	self.nos_plus -= 0.85

		if msg.buttons[6]== 1:		self.avel_l = -msg.axes[4] #L2
		elif msg.buttons[6] == 0:	self.avel_l = 0.0

		if msg.buttons[7] == 1:		self.avel_r = msg.axes[5] #R2
		elif msg.buttons[7] == 0: 	self.avel_r = 0.0			

		self.vel_z = (self.avel_r + self.avel_l) * self.ang_coeff
		self.nos_plus = clip(self.nos_plus, 4, 9)
		self.nos = clip(self.v_quintic(self.nos_plus), 18, 100)

if __name__ == '__main__':
	rospy.init_node('exkinematics_node')
	rospy.logwarn('kinematics node is running')
	example_ = kinematics()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
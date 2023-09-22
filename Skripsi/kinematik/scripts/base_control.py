#!/usr/bin/env python

import rospy
import time
from kinematik.math_func import *
from numpy import radians, matrix, pi, array, dot
from math import atan2
from geometry_msgs.msg import Vector3
from kinematik.msg import omni
from sensor_msgs.msg import Joy

class base_control:
	#param inv
	alpha = radians(rospy.get_param("/kinematik/roda6_wheel_alpha"))
	gamma = radians(rospy.get_param("/kinematik/roda6_wheel_gamma"))
	l = 0.43
	lam_ = 0.55


	lam_2 = 2 * lam_
	vel2rpm = (60 / (pi * 0.1))
	#ros object
	start = rospy.Time.from_sec(time.time()).to_sec()
	last = rospy.Time.from_sec(time.time()).to_sec()
	dt = 0
	rpm = 160
	#msg
	test = matrix([[0, 0, 0]], dtype='float32').T
	omni_speed = omni()	
	threshold = 0.75
	th = 0
	e_pose_th = matrix([[0, 0, 0]], dtype='float32').T
	e_pose = matrix([[0, 0, 0]], dtype='float32').T
	target = matrix([[0, 0, 0]], dtype='float32').T

	def __init__(self):				
		self.motor_pub = rospy.Publisher('/robot/motor', omni, queue_size=10)
		self.target_sub = rospy.Subscriber('/robot/target', Vector3, self.target_callback, queue_size=5)
		self.pose_sub = rospy.Subscriber('/robot/pose', Vector3, self.pose_callback, queue_size=5)		
		self.timer_pub = rospy.Timer(rospy.Duration(1.0 / 50.0), self.motor_pwm_pub) #50Hz			

	def target_callback(self, msg):
		self.target[0, 0] = msg.x
		self.target[1, 0] = msg.y
		self.target[2, 0] = deg_to_rad(msg.z)
		# print_error(self.target[0, 0], self.target[1, 0], self.target[2, 0])

	def pose_callback(self, msg):	
		self.th = msg.z
		self.e_pose[0, 0] = (self.target[0, 0] - msg.x) 
		self.e_pose[1, 0] = (self.target[1, 0] - msg.y) 
		self.e_pose[2, 0] = (self.target[2, 0] - msg.z)
		
		print_error(self.e_pose[0, 0], self.e_pose[1, 0], rad_to_deg(self.e_pose[2, 0]))
		
		self.e_pose[0, 0] *= self.lam_2
		self.e_pose[1, 0] *= self.lam_2
		self.e_pose[2, 0] *= self.lam_
		self.e_pose = threshold_error(self.e_pose, self.threshold)

	def motor_pwm_pub(self, event=None):
		cmd_vel = inv_rot_omni6(self.th) * (self.e_pose) * self.vel2rpm		
		wheel_speed = dot(jacobian_omni6(self.alpha, self.gamma, self.l), cmd_vel)

		self.omni_speed.w1 = clip_rpm(wheel_speed[0, 0], self.rpm) 
		self.omni_speed.w2 = clip_rpm(wheel_speed[1, 0], self.rpm) 
		self.omni_speed.w3 = clip_rpm(wheel_speed[2, 0], self.rpm) 
		self.omni_speed.w4 = clip_rpm(wheel_speed[3, 0], self.rpm) 
		self.omni_speed.w5 = clip_rpm(wheel_speed[4, 0], self.rpm) 
		self.omni_speed.w6 = clip_rpm(wheel_speed[5, 0], self.rpm) 

		self.motor_pub.publish(self.omni_speed)
		#print(f'x = {self.e_pose[0, 0]}		y = {self.e_pose[1, 0]}		z = {self.e_pose[2, 0]}')

if __name__ == '__main__':
	rospy.init_node("base_control_node")
	rospy.logwarn("base_control_node has started")
	base = base_control()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass




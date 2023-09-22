#!/usr/bin/env python

import rospy
import time
from kinematik.math_func import *
from numpy.linalg import pinv
from numpy import radians, matrix, pi, degrees, array
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from kinematik.msg import enc

class odom:
	#param odom
	alpha = radians(rospy.get_param("/kinematik/odom_alpha"))
	gamma = radians(rospy.get_param("/kinematik/odom_gamma"))
	l = 0.45
	x = [0.42, -0.13]
	y = [-0.2, 0.44]
	
	#ros object
	start = rospy.Time.from_sec(time.time()).to_sec()
	last = rospy.Time.from_sec(time.time()).to_sec()
	dt = 0		
	#init variable odom
	yaw_vel : float = 0
	yaw_rad : float = 0
	l_w1 = 0
	l_w2 = 0
	pose = matrix([[0, 0, 0]], dtype='float32').T
	w = matrix([[0, 0]], dtype='float32').T
	last_w = matrix([[0, 0]], dtype='float32').T
	
	def __init__(self):		
		self.odom_sub = rospy.Subscriber('/robot/enc_ext', enc, self.odom_callback, queue_size=1)
		self.bno_sub = rospy.Subscriber('/imu/euler', Vector3, self.imu_callback, queue_size=10)		
		self.pose_pub = rospy.Publisher('/robot/pose', Vector3, queue_size=10)		
		self.timer_pub = rospy.Timer(rospy.Duration(1.0 / 50.0), self.compute_cmd_vel) #50Hz

	def compute_cmd_vel(self, event=None):
		j_odom2 = jacobian_odom2(self.alpha, self.gamma, self.l)
		iv_rot = inv_rot_odom2(self.yaw_rad)
		j_fwd = pinv(j_odom2 * iv_rot)
		pose_dot = j_fwd * self.w

		self.pose[0, 0] = around(pose_dot[0, 0], decimals=3)
		self.pose[1, 0] = around(pose_dot[1, 0], decimals=3)
		self.pose[2, 0] = around(self.yaw_rad, decimals=6)

		#position robot
		self.robot_pose = Vector3(self.pose[0, 0], self.pose[1, 0], self.pose[2, 0])		
		self.pose_pub.publish(self.robot_pose)

	def imu_callback(self, msg):			
		self.yaw_rad = rad_(msg.z)
		if self.yaw_rad > 0: self.yaw_rad -= pi
		elif self.yaw_rad < 0: self.yaw_rad += pi

	def odom_callback(self, enc):												
		self.w[0, 0] = enc.w1
		self.w[1, 0] = enc.w2
		

if __name__ == '__main__':
	rospy.init_node("odometry_node")
	rospy.logwarn("odom node has started")
	odometry = odom()
	try:
		rospy.spin()	
	except rospy.ROSInterruptException:
		pass

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from master_krai.msg import mekanisme
from fungsi.math_func import *
from scipy.special import expit as ep #sigmoid
from math import ceil
from numpy import clip, rint

class example():
	m = mekanisme()
	prev_joy = Joy()
	vel_chain : float = 0
	lift : float = 0

	def __init__(self):
		self.PS4_joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)
		self.mekanisme_pub = rospy.Publisher('/robot/mekanisme', mekanisme, queue_size=10)
		self.timer_pub = rospy.Timer(rospy.Duration(1.0 / 50.0), self.joy_pub) #50Hz

	def joy_cb(self, msg):
		#toggle cmd
		if msg.buttons != self.prev_joy.buttons:
			self.prev_joy = msg
			if msg.buttons[0]:	self.m.silang = not self.m.silang
			if msg.buttons[1]:	self.m.bulat = not self.m.bulat
			if msg.buttons[2]:	self.m.wajik = not self.m.wajik
		
		#button cmd
		if msg.buttons[3] == 1: self.m.kotak == True
		elif msg.buttons[3] == 0: self.m.kotak == False

		#--------------------mekanisme yg pake belt/rack pinion----------------------#
		if msg.buttons[4] == 1 :	#R1			
			self.vel_chain += 0.025
			self.lift = 700 ** ep(self.vel_chain)
		if msg.buttons[5] == 1 : #L1	
			self.vel_chain += 0.025
			self.lift = -700 ** ep(self.vel_chain)
		elif msg.buttons[4] == 0 and msg.buttons[5] == 0:
			self.lift = 0
			self.vel_chain = 0
		self.vel_chain = clip(self.vel_chain, 0, 10)
		self.m.belt = ceil(rint(clip(self.lift, -700, 700)))	

	def joy_pub(self, event=None):
		self.mekanisme_pub.publish(self.m)

if __name__ == '__main__':
	rospy.init_node('example_node')
	rospy.logwarn('example node is running')
	example_ = example()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import JoyFeedback
from sensor_msgs.msg import Joy
from master_krai.msg import Mechanism
from std_msgs.msg import Int8
from scipy.special import expit as ep
import numpy as np
import math

class mekanisme_gajah():
	pwm_bldc_1 = rospy.get_param("/Mekanisme/ring_mri_1")
	pwm_bldc_2 = rospy.get_param("/Mekanisme/ring_mri_2")
	min_pwm = rospy.get_param("/Mekanisme/min_pwm")
	max_pwm = rospy.get_param("/Mekanisme/max_pwm")
	offset_1 = rospy.get_param("/Mekanisme/offset_bldc_1")	
	offset_2 = rospy.get_param("/Mekanisme/offset_bldc_2")
	boost_1 = rospy.get_param("/Mekanisme/boost_bldc_1")
	boost_2 = rospy.get_param("/Mekanisme/boost_bldc_2")	

	mechanism = Mechanism()
	prev_joy_msg = Joy()	
	
	bldc_coeff : float =  1.5
	bldc_sum_1 : float = 0		
	bldc_sum_2 : float = 0		
	bldc_offset_1 : float = 0
	bldc_offset_2 : float = 0	
	bldc_in_1 : float =  min_pwm
	bldc_in_2 : float = min_pwm
	nos_1 : float = 0
	nos_2 : float = 0
	vertical : float = 0
	horizontal : float = 0
	chain_lift : float = 0	
	chain_lift2 : float = 0
	vel_chain2 : float = 0
	vel_chain : float = 0
	shoot_mode : bool = False		
	i : int = 0

	def __init__(self):
		self.PS4_joy_sub = rospy.Subscriber('/joy', Joy, self.Command_cb, queue_size = 1)		
		self.mekanisme_pub = rospy.Publisher('/Gajah/Mechanism', Mechanism, queue_size = 1)

	def pub_mechanism(self):
		self.mechanism.y_penembak = math.ceil(np.rint(np.clip(self.vertical, -700, 700)))
		self.mechanism.x_penembak = math.ceil(np.rint(np.clip(self.horizontal, -700, 700)))
		self.mechanism.lift_gripper = math.ceil(np.rint(np.clip(self.chain_lift, -500, 500)))
		self.mechanism.roller_spd_1 = math.ceil(np.rint(np.clip(self.bldc_in_1, self.min_pwm, self.max_pwm)))
		self.mechanism.roller_spd_2 = math.ceil(np.rint(np.clip(self.bldc_in_2, self.min_pwm, self.max_pwm)))
		self.mechanism.lift_gripper2 = math.ceil(np.rint(np.clip(self.chain_lift2, -700, 700)))
		self.mekanisme_pub.publish(self.mechanism)

	def Command_cb(self, msg):
		if msg.buttons != self.prev_joy_msg.buttons:
			self.prev_joy_msg = msg
			if msg.buttons[10]:
				self.mechanism.wajik = not self.mechanism.wajik
			if msg.buttons[10]:
				self.shoot_mode = not self.shoot_mode
			if msg.buttons[0]:	#reload
				self.mechanism.silang = not self.mechanism.silang
			if msg.buttons[1]:	#capit
				self.mechanism.bulat = not self.mechanism.bulat
			if msg.buttons[16]: #atas
				self.i += 1			
				self.bldc_offset_1 = self.bldc_offset_2 = 0
			if msg.buttons[15]: #bawah
				self.i -= 1	
				self.bldc_offset_1 = self.bldc_offset_2 = 0				

		self.i = np.clip(self.i, 1, len(self.pwm_bldc_1))

		#--------------------lontar ring-------------------------#				
		if msg.buttons[3] == 1:
			self.mechanism.kotak = True
			self.nos_1 = self.boost_1[self.i-1]
			self.nos_2 = self.boost_2[self.i-1]
		elif msg.buttons[3] == 0:
			self.mechanism.kotak = False
			self.nos_1 = 0
			self.nos_2 = 0

		#---------------------arah shooter-----------------------#
		self.vertical = msg.axes[3] * -400			
		if msg.buttons[14] == 1: self.horizontal = -120	#kanan			
		if msg.buttons[13] == 1: self.horizontal = 120	#kiri
		elif msg.buttons[14] == 0 and msg.buttons[13] == 0:	self.horizontal = 0			

		#--------------------chain gripper----------------------#
		if msg.buttons[4] == 1 :	#R1			
			self.vel_chain += 0.025
			self.chain_lift = 700 ** ep(self.vel_chain)
		if msg.buttons[5] == 1 : #L1	
			self.vel_chain += 0.025
			self.chain_lift = -700 ** ep(self.vel_chain)
		elif msg.buttons[5] == 0 and msg.buttons[4] == 0:
			self.chain_lift = 0
			self.vel_chain = 0
		self.vel_chain = np.clip(self.vel_chain, 0, 10)
		self.chain_lift2 = 0				

		#-----------------------shoot off mode----------------------#
		if self.shoot_mode == False:
			self.bldc_in_1 -= 8
			self.bldc_in_2 -= 16
										
		#--------------------shoot on mode--------------------------#
		elif self.shoot_mode == True:									
			if msg.buttons[8] == 1:	#share 
				self.bldc_offset_1 -= 0.25
				self.bldc_offset_2 -= 0.25
			if msg.buttons[9] == 1:	#options
				self.bldc_offset_1 += 0.25
				self.bldc_offset_2 += 0.5		

			self.bldc_in_1 += 16
			self.bldc_in_2 += 8
		
		if msg.buttons[11] == 1:	
			self.bldc_in_1 = 4600
			self.bldc_in_2 = 4800

		elif msg.buttons[11] == 0:				
			self.bldc_offset_1 = np.clip(self.bldc_offset_1, -self.offset_1, self.offset_1)
			self.bldc_offset_2 = np.clip(self.bldc_offset_2, -self.offset_2, self.offset_2)
			self.bldc_sum_1 = (self.bldc_offset_1 + self.pwm_bldc_1[self.i - 1])
			self.bldc_sum_2 = ((self.bldc_offset_2 * self.bldc_coeff) + self.pwm_bldc_2[self.i - 1])
			self.bldc_in_1 = np.clip(self.bldc_in_1 , self.min_pwm, self.bldc_sum_1) + self.nos_1
			self.bldc_in_2 = np.clip(self.bldc_in_2, self.min_pwm, self.bldc_sum_2) + self.nos_2
		#print(f'r1 = {self.bldc_in_1}	r2 = {self.bldc_in_2}')		


if __name__ == '__main__':
	try:
		rospy.init_node('Mekanisme_gajah', anonymous = True)
		rospy.logwarn("Mekanisme gajah node is running")
		mekanisme_obj = mekanisme_gajah()		
		
		rate = rospy.Rate(200)  # 10hz
		while not rospy.is_shutdown():
			mekanisme_obj.pub_mechanism()
			rate.sleep()
			
	except rospy.ROSInterruptException:
		ros.logwarn("Mekanisme node is stop running...")

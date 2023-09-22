#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import JoyFeedback
from sensor_msgs.msg import Joy
from master_krai.msg import Mechanism
from scipy.special import expit as ep
import numpy as np
import math

class mekanisme_gajah():
	pwm_bldc_1 = rospy.get_param("/Mekanisme/pwm_bldc_1")
	pwm_bldc_2 = rospy.get_param("/Mekanisme/pwm_bldc_2")
	min_pwm_1 = rospy.get_param("/Mekanisme/min_pwm_1")
	max_pwm_1 = rospy.get_param("/Mekanisme/max_pwm_1")
	min_pwm_2 = rospy.get_param("/Mekanisme/min_pwm_2")
	max_pwm_2 = rospy.get_param("/Mekanisme/max_pwm_2")
	offset_1 = rospy.get_param("/Mekanisme/offset_bldc_1")	
	offset_2 = rospy.get_param("/Mekanisme/offset_bldc_2")	

	mechanism = Mechanism()
	prev_joy_msg = Joy()	
	
	bldc_coeff : float =  1.5
	bldc_sum_1 : float = 0		
	bldc_sum_2 : float = 0		
	bldc_offset_1 : float = 0
	bldc_offset_2 : float = 0	
	bldc_in_1 : float =  min_pwm_1
	bldc_in_2 : float = min_pwm_2
	vertical : float = 0
	horizontal : float = 0
	chain_lift : float = 0	
	chain_lift2 : float = 0
	vel_chain2 : float = 0
	vel_chain : float = 0
	shoot_mode : bool = False		
	i : int = 0
	
	def __init__(self):
		self.PS4_joy_sub = rospy.Subscriber('/joy', Joy, self.Command_cb)
		self.mekanisme_pub = rospy.Publisher('/Gajah/Mechanism', Mechanism, queue_size = 1)	

	def pub_mechanism(self):
		self.mechanism.y_penembak = math.ceil(np.rint(np.clip(self.horizontal, -700, 700)))
		self.mechanism.x_penembak = math.ceil(np.rint(np.clip(self.vertical, -700, 700)))
		self.mechanism.lift_gripper = math.ceil(np.rint(np.clip(self.chain_lift, -500, 500)))
		self.mechanism.roller_spd_1 = math.ceil(np.rint(self.bldc_in_1))
		self.mechanism.roller_spd_2 = math.ceil(np.rint(self.bldc_in_2))
		self.mechanism.lift_gripper2 = math.ceil(np.rint(np.clip(self.chain_lift2, -700, 700)))
		self.mekanisme_pub.publish(self.mechanism)	

	def Command_cb(self, msg):
		if msg.buttons != self.prev_joy_msg.buttons:
			self.prev_joy_msg = msg
			if msg.buttons[10]:
				self.shoot_mode = not self.shoot_mode
			if msg.buttons[0]:	#reload
				self.mechanism.silang = not self.mechanism.silang
			if msg.buttons[1]:	#capit
				self.mechanism.bulat = not self.mechanism.bulat
			if msg.buttons[2]:	#segitiga
				self.mechanism.wajik = not self.mechanism.wajik							
			if msg.buttons[16]: #atas
				self.i += 1	
				self.bldc_offset_1 = self.bldc_offset_2 = 0							
			if msg.buttons[15]: #bawah
				self.i -= 1				
				self.bldc_offset_1 = self.bldc_offset_2 = 0						

		#--------------------lontar ring-------------------------#
		if msg.buttons[3] == 1:	self.mechanism.kotak = True
		elif msg.buttons[3] == 0:	self.mechanism.kotak = False

		#--------------------arah shooter------------------------#
		if msg.buttons[13] == 1: self.horizontal = -80 	#kanan
		if msg.buttons[14] == 1: self.horizontal = 80	#kiri
		elif msg.buttons[13] == 0 and msg.buttons[14] == 0:	self.horizontal = 0
					
		self.vertical = msg.axes[3] * -100		

		#-----------------------shoot off mode----------------------#
		if self.shoot_mode == False:						
			if msg.buttons[4] == 1 :	#R1			
				self.vel_chain += 0.1
				self.chain_lift = -170 ** ep(self.vel_chain)
			if msg.buttons[5] == 1 : #L1	
				self.vel_chain += 0.1
				self.chain_lift = 170 ** ep(self.vel_chain)
			elif msg.buttons[4] == 0 and msg.buttons[5] == 0:
				self.chain_lift = 0
				self.vel_chain = 0
			self.chain_lift2 = 0
			self.vel_chain2 = 0
			self.vel_chain = np.clip(self.vel_chain, 0, 10)
 
			#rem bldc
			self.bldc_in_1 -= 6
			self.bldc_in_2 -= 6
			
		#--------------------shoot on mode--------------------------#
		elif self.shoot_mode == True:						
			if msg.buttons[8] == 1:	#share 
				self.bldc_offset_1 -= 0.5
				self.bldc_offset_2 -= 0.75
			if msg.buttons[9] == 1:	#options
				self.bldc_offset_1 += 0.5
				self.bldc_offset_2 += 1.25

			if msg.buttons[4] == 1 :	#R1			
				self.vel_chain2 += 0.1
				self.chain_lift2 = 700 ** ep(self.vel_chain2)
			if msg.buttons[5] == 1 : #L1	
				self.vel_chain2 += 0.1
				self.chain_lift2 = -700 ** ep(self.vel_chain2)
			elif msg.buttons[4] == 0 and msg.buttons[5] == 0:
				self.chain_lift2 = 0
				self.vel_chain2 = 0				
			self.vel_chain = 0
			self.chain_lift = 0
			self.vel_chain2 = np.clip(self.vel_chain2, 0, 10)

			#accel bldc
			self.bldc_in_1 += 10
			self.bldc_in_2 += 10
				
		if msg.buttons[12] == 1:	
			self.bldc_in_1 = 4500
			self.bldc_in_2 = 4500
		elif msg.buttons[12] == 0:
			self.i = np.clip(self.i, 1, len(self.pwm_bldc_1))
			self.bldc_offset_1 = np.clip(self.bldc_offset_1, -self.offset_1, self.offset_1)
			self.bldc_offset_2 = np.clip(self.bldc_offset_2, -self.offset_2, self.offset_2)
			self.bldc_sum_1 = (self.bldc_offset_1 + self.pwm_bldc_1[self.i - 1])
			self.bldc_sum_2 = ((self.bldc_offset_2 * self.bldc_coeff) + self.pwm_bldc_2[self.i - 1])
			self.bldc_in_1 = np.clip(self.bldc_in_1 , self.min_pwm_1, self.bldc_sum_1)
			self.bldc_in_2 = np.clip(self.bldc_in_2, self.min_pwm_2, self.bldc_sum_2)

		print(f'bldc 1 = {self.bldc_in_1}')

if __name__ == '__main__':
	try:
		rospy.init_node('Mekanisme_kelinci', anonymous = True)
		rospy.logwarn("Mekanisme kelinci node is running")
		mekanisme_obj = mekanisme_gajah()		
		rate = rospy.Rate(200)  # 10hz
		while not rospy.is_shutdown():
			mekanisme_obj.pub_mechanism()
			rate.sleep()
			
	except rospy.ROSInterruptException:
		ros.logwarn("Mekanisme node is stop running...")
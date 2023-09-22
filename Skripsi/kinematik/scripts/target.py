#!/usr/bin/env python

import rospy
import sys, os
from kinematik.math_func import *
from geometry_msgs.msg import Vector3


def fx(x0, x1, x2, x3, t):
	a1 = x0 * ((1 -t ) ** 3)
	a2 = 3 * x1 * t * ((1 - t) ** 2)
	a3 = 3 * x2 * (t ** 2) * (1 - t)
	a4 = x3 * (t ** 3)
	return a1 + a2 + a3 + a4

def gx(x0, x1, x2, x3, t):	
	a1 = x0 * ((1 -t ) ** 3)
	a2 = 3 * x1 * t * ((1 - t) ** 2)
	a3 = 3 * x2 * (t ** 2) * (1 - t)
	a4 = x3 * (t ** 3)
	return a1 + a2 + a3 + a4

def my_node(myArg1,myArg2,myArg3):
	rospy.init_node('target', anonymous=True)
	ub = rospy.Publisher('/robot/target', Vector3, queue_size=10)
	vec = Vector3()
	rate = rospy.Rate(50) # 10hz
	try:
		while not rospy.is_shutdown():
			vec.x = myArg1
			vec.y = myArg2
			vec.z = myArg3
			ub.publish(vec)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass

if __name__=="__main__":
	argv = rospy.myargv()
	if len(argv) == 4:
		my_node(float(argv[1]), float(argv[2]), float(argv[3]))
	else:
		print("usage: my_node.py arg1 arg2")

#!/usr/bin/env python3

from numpy import pi, cos, sin, matrix, around, clip, reshape, pi, radians, sqrt, fabs, degrees, allclose, poly1d
from numpy.linalg import pinv, norm

#konversi dari 0 - 2pi ke -pi - pi
def rad_(angle): 
	if angle >= (pi):
		return around((-1.0 * (angle % (pi))), decimals=6)
	else:
		return around(angle, decimals=6)

def deg_(angle):
	if angle >= 180.0:
		return around((-1.0 * (angle % (180.0))), decimals=8)
	else:
		return around(angle, decimals=8)

def deg_to_rad(angle):
	_rad = around(radians(angle), decimals=6)
	return rad_(_rad)

def rad_to_deg(angle):
	_deg = around(degrees(angle), decimals=6)
	return deg_(_deg)

#rounding floating point number error DALAM RADIAN
def sin_(angle):
	if (allclose(0, sin(angle), atol=1e-04) == True):
		return 0
	else:
		return around(sin(angle), decimals=6)

def cos_(angle):
	if (allclose(0, cos(angle), atol=1e-04) == True):
		return 0
	else:
		return around(cos(angle), decimals=6)

#inverse matriks rotasi
def inv_rot(th):
	return matrix([
		[cos_(th), sin_(th), 0], 
		[-sin_(th), cos_(th), 0], 
		[0., 0., 1.]], dtype='float32')

#--------- ODOMETRY -------------#

def jacobian_odom3(robot_alpha, robot_gamma, l):
	j_inv = matrix([[0, 0, 0,], [0, 0, 0], [0, 0, 0]], dtype='float32')
	a = robot_alpha.reshape((len(robot_alpha), 1))
	y = robot_gamma.reshape((len(robot_gamma), 1))
	j_inv[:, 0:1] = cos_(a)
	j_inv[:, 1:2] = sin_(a)
	j_inv[:, 2:3] = l * sin_((a - y))
	return j_inv

#---------INV KINEMATICS------------#

def jacobian_omni4(alpha, gamma, l):
	ji_omni4 = matrix([[0, 0, 0,], [0, 0, 0], [0, 0, 0], [0, 0, 0]], dtype='float32')
	a = alpha.reshape((len(alpha), 1))
	y = gamma.reshape((len(alpha), 1))
	ji_omni4[:, 0:1] = cos_(a)
	ji_omni4[:, 1:2] = sin_(a)
	ji_omni4[:, 2:3] = l * sin_((a - y))
	return ji_omni4

def vel_vector(x, y, z):
	vect = matrix([[0., 0., 0.]], dtype='float32').T
	vect[0, 0] = x
	vect[0, 1] = y
	vect[0, 2] = z
	return vel_vector

#in_range dan out_range adalah list isi 2
def interp_(val, in_range, out_range): 
	res = clip(val, in_range[0], in_range[1])
	res = interp(res, in_range, out_range)

def threshold_error(error, thres):	
	zeros = matrix([[0, 0, 0]], dtype='float32').T
	thres_xy = norm(error)
	if thres_xy < thres:
		return zeros
	else:
		return around(error, decimals=5)


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

#rounding floating point number error dalam radian
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


#--------- ODOMETRY -------------#

#inverse matriks rotasi
def inv_rot_odom2(th):
	return matrix([
		[cos_(th), sin_(th), 0], 
		[-sin_(th), cos_(th), 0], 
		[0., 0., 1.]], dtype='float32')


def jacobian_odom2(robot_alpha, robot_gamma, l):
	j_inv = matrix([[0, 0, 0,], [0, 0, 0], [0, 0, 0]], dtype='float32')
	j_inv[0, 0] = cos_(robot_alpha[0])
	j_inv[0, 1] = sin_(robot_alpha[0])
	j_inv[0, 2] = l * sin_(robot_alpha[0] - robot_gamma[0])
	j_inv[1, 0] = cos_(robot_alpha[1])
	j_inv[1, 1] = sin_(robot_alpha[1])
	j_inv[1, 2] = l * sin_((robot_alpha[1] - robot_gamma[1]))
	j_inv[2, 0] = 0
	j_inv[2, 1] = 0
	j_inv[2, 2] = 0
	return around(j_inv, decimals=6)


def jacobian_odom2a(robot_alpha, robot_gamma, x, y):
	j_inv = matrix([[0, 0, 0,], [0, 0, 0], [0, 0, 0]], dtype='float32')
	j_inv[0, 0] = cos_(robot_alpha[0])
	j_inv[0, 1] = sin_(robot_alpha[0])
	j_inv[0, 2] = (x[0] * sin_(robot_alpha[0])) - (y[0] * (cos_(robot_alpha[0]))) 
	j_inv[1, 0] = cos_(robot_alpha[1])
	j_inv[1, 1] = sin_(robot_alpha[1])
	j_inv[1, 2] = (x[1] * sin_(robot_alpha[0])) - (y[1] * (cos_(robot_alpha[1]))) 
	j_inv[2, 0] = 0
	j_inv[2, 1] = 0
	j_inv[2, 2] = 0
	return around(j_inv, decimals=6)


def jacobian_odom2_(alpha, gamma, l):
	ji_omni6 = matrix([[0, 0, 0,], [0, 0, 0]], dtype='float32')
	a = alpha.reshape((len(alpha), 1))
	y = gamma.reshape((len(alpha), 1))
	ji_omni6[:, 0:1] = cos_(a)
	ji_omni6[:, 1:2] = sin_(a)
	ji_omni6[:, 2:3] = l * sin_((a - y))
	return around(ji_omni6, decimals=6)

def inv_rot_odom2_(th):
	return matrix([
		[cos_(th), sin_(th)], 
		[-sin_(th), cos_(th)], 
		[0., 0]], dtype='float32')

#---------INV KINEMATICS------------#

def inv_rot_omni6(th):
	return matrix([
		[cos_(th), sin_(th), 0.], 
		[-sin_(th), cos_(th), 0.], 
		[0., 0., 1.]], dtype='float32')

def jacobian_omni6(alpha, gamma, l):
	ji_omni6 = matrix([[0, 0, 0,], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]], dtype='float32')
	a = alpha.reshape((len(alpha), 1))
	y = gamma.reshape((len(alpha), 1))
	ji_omni6[:, 0:1] = cos_(a)
	ji_omni6[:, 1:2] = sin_(a)
	ji_omni6[:, 2:3] = l * sin_((a - y))
	return around(ji_omni6, decimals=3)

def clip_rpm(speed, rpm):
	return around(clip(speed, -rpm, rpm), decimals=1)

def threshold_error(error, thres):	
	zeros = matrix([[0, 0, 0]], dtype='float32').T
	thres_xy = norm(error)
	if thres_xy < thres:
		return zeros
	else:
		return around(error, decimals=5)

def gain_factor_pose(error, L):	
	ut = matrix([[0, 0, 0]], dtype='float32').T	
	error[2, 0] = deg_to_rad(error[2, 0])
	ut[0, 0] = error[0, 0] * (L ** -fabs(error[0, 0]))
	ut[1, 0] = error[1, 0] * (L ** -fabs(error[1, 0]))
	ut[2, 0] = error[2, 0] * (L ** -fabs(error[2, 0]))
	return around(ut, decimals=3)

def covariance_pose(error, mat_cov): #mat cov 3x3
	ut = matrix([[0, 0, 0]], dtype='float32').T		
	ut = mat_cov * error
	return around(ut, decimals=3)

def fx(x0, x1, x2, x3, t):
	a1 = x0 * ((1 -t ) ** 3)
	a2 = 3 * x1 * t * ((1 - t) ** 2)
	a3 = 3 * x2 * (t ** 2) * (1 - t)
	a4 = x3 * (t ** 3)
	return a1 + a2 + a3 + a4

def lamda_(error, l):
	a = l
	a1 = l/2
	a2 = l/4
	a3 = l * 5
	ut = matrix([[0, 0, 0]], dtype='float32').T		
	ut[0, 0] = fx(0., a, a1, a2, fabs(error[0 ,0])) * a3
	ut[1, 0] = fx(0., a, a1, a2, fabs(error[1 ,0])) * a3
	ut[2, 0] = a * (fabs(error[2, 0])) * a3
	return around(ut, decimals=3)

def es(now, last, a):
	out = (a * now) + ((1 - a) * last)
	return around(out, decimals=6)

def print_error(x, y, z):
	print(f'Ex = {around(x, decimals=3)} m\t Ey = {around(y, decimals=3)} m\t Ez = {around(z, decimals=3)}\xb0')

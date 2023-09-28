#program helper untuk menyederhanakan fwd kinematics

from numpy import matrix, sin, cos, around, pi, array
from numpy.linalg import pinv, inv
from sympy import *

init_printing()
var('q1 q2 q3')
var('Xg, Yg, Zg')
cs = Symbol("cos(th)")
sn = Symbol("sin(th)")

a = [0., pi, -pi/2.0]
g = [-pi/2, pi/2, pi]
l = 10

J_odom3 = Matrix([
	[cos(a[0]), sin(a[0]), l * sin(a[0] - g[0])],
	[cos(a[1]), sin(a[1]), l * sin(a[1] - g[1])],
	[cos(a[2]), sin(a[2]), l * sin(a[2] - g[2])]
	])

Rot = Matrix([
	[cs, -sn, 0],
	[sn, cs, 0],
	[0, 0, 1]
	])

enc = Matrix([q1, q2, q3])
pose = Matrix([Xg, Yg, Zg])

e = J_odom3.inv().multiply(Rot)
# pprint(e.multiply(enc))
e2 = e.multiply(enc)
# pprint(J_odom3.inv())
print("\n")
pprint(Eq(pose, e2))
# a = array([0., pi, -pi/2.0])
# g = array([-pi/2, pi/2, pi])

# def jacobian_odom2(alpha, gamma, l):
# 	ji_omni6 = matrix([[0, 0, 0,], [0, 0, 0], [0, 0, 0]], dtype='float32')
# 	a = alpha.reshape((len(alpha), 1))
# 	y = gamma.reshape((len(alpha), 1))
# 	ji_omni6[:, 0:1] = cos(a)
# 	ji_omni6[:, 1:2] = sin(a)
# 	ji_omni6[:, 2:3] = l * sin((a - y))
# 	return around(ji_omni6, decimals=6)

# res = jacobian_odom2(a, g, 70)
# print(res)
# print('\n')
# print(inv(res))
# print('\n')
# print(pinv(res))

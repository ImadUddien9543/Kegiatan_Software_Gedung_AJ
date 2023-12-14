#program helper untuk menyederhanakan fwd kinematics dan inv kinematics
#jalankan di terminal
#requirements:
from numpy import matrix, sin, cos, around, pi, array
from numpy.linalg import pinv, inv
from sympy import *

class color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'


init_printing()
var('q1 q2 q3')
var('Xg, Yg, Zg')
var('w1 w2 w3 w4')
cs = Symbol("cos(th)")
sn = Symbol("sin(th)")

a = [0., pi, -pi/2.0]
g = [-pi/2, pi/2, pi]
l = 10

a_ = [pi/4, 3*pi/4, -3*pi/4, -pi/4]
g_ = [-pi/4, pi/4, 3*pi/4, -3*pi/4]
l_ = 10

enc = Matrix([q1, q2, q3])
pose = Matrix([Xg, Yg, Zg])
w = Matrix([w1, w2, w3, w4])

J_odom3 = Matrix([
	[cos(a[0]), sin(a[0]), l * sin(a[0] - g[0])],
	[cos(a[1]), sin(a[1]), l * sin(a[1] - g[1])],
	[cos(a[2]), sin(a[2]), l * sin(a[2] - g[2])]
	])

J_base4 = Matrix([
	[cos(a_[0]), sin(a_[0]), l * sin(a_[0] - g_[0])],
	[cos(a_[1]), sin(a_[1]), l * sin(a_[1] - g_[1])],
	[cos(a_[2]), sin(a_[2]), l * sin(a_[2] - g_[2])],
	[cos(a_[3]), sin(a_[3]), l * sin(a_[3] - g_[3])]
	])

Rot = Matrix([
	[cs, -sn, 0],
	[sn, cs, 0],
	[0, 0, 1]
	])

IRot = Matrix([
	[cs, sn, 0],
	[-sn, cs, 0],
	[0, 0, 1]
	])

f = J_odom3.inv().multiply(Rot)
f2 = f.multiply(enc)
print(color.CYAN + '\tFORWARD KINEMATICS\n' + color.END)
pprint(Eq(pose, f2))

print(color.CYAN + '\n\tINVERSE KINEMATICS\n' + color.END)
i = J_base4.multiply(IRot)
i2 = i.multiply(pose)
pprint(Eq(w, i2))

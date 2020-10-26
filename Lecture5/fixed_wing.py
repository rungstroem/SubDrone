#!/usr/bin/env python3

# load modules
from math import pi
import matplotlib.pyplot as plt
from pylab import *
import numpy as np
import os 

# implementation of a cubic hermite spline
f = open("convert_u/UTM_positions.csv", "r")
ff = open("fixed_wing.csv","w")


#global vars
points = []
count = 0
leftovers = []
p0 = []
pn = []
f_len = os.path.getsize('convert_u/UTM_positions.csv')
#Homademade
def write_to_file(points_list):
	ff.write(str(points_list[0]))
	ff.write(",")
	ff.write(str(points_list[1]))
	ff.write(",")
	ff.write(str(points_list[2]))
	ff.write(",")
	ff.write(str(points_list[3]))
	ff.write(",")
	ff.write(str(points_list[4]))
	ff.write(",")
	ff.write(str(points_list[5]))
	pass
	
def tangent_cal(p0,p1,a):
	# a can be any value between 0 and 1 - determines the tightness of the curve
	delta1 = a* (p1[0] -p0[0])
	delta2 = a* (p1[1] -p0[1])  
	T = [delta1, delta2]
	return T

#taken from Kjeld
class cubic_hermite_spline():
	def __init__(self):
		pass

	def v2d_scalar_mul (self, v, f):
		return [v[0]*f, v[1]*f]

	def v2d_add (self, v1, v2):
		return [v1[0]+v2[0], v1[1]+v2[1]]

	def goto_wpt (self, p1, t1, p2, t2, steps):
		# http://cubic.org/docs/hermite.htm
		# http://en.wikipedia.org/wiki/Cubic_Hermite_spline#Interpolation%20on%20a%20single%20interval
		p = []
		for t in range(steps):
			s = t/(steps * 1.0) # scale s to go from 0 to 1

			# # calculate basis function
			h1 = 2*s**3 - 3*s**2 + 1
			h2 = -2*s**3 + 3*s**2
			h3 = s**3 - 2*s**2 + s
			h4 = s**3 - s**2

			# multiply and sum functions together to build the interpolated point along the curve
			v1 = self.v2d_scalar_mul(p1,h1)
			v2 = self.v2d_scalar_mul(p2,h2)
			v3 = self.v2d_scalar_mul(t1,h3)
			v4 = self.v2d_scalar_mul(t2,h4)

			p.append(self.v2d_add(self.v2d_add(self.v2d_add(v1,v2),v3),v4))
		return p


for line in f:

	csv = line.split(",")
	
	if(count == 0):
		p = []
		p.append(csv[0])
		p.append(csv[1])
		p.append(csv[2])
		p.append(csv[3])
		p.append(csv[4])
		p.append(csv[5])
		write_to_file(p)
		p0 = [float(csv[0]),float(csv[1])]
	elif(count == f_len):
		p = []
		p.append(csv[0])
		p.append(csv[1])
		p.append(csv[2])
		p.append(csv[3])
		p.append(csv[4])
		p.append(csv[5])
		write_to_file(p)
		pn = [float(csv[0]),float(csv[1])]
	else:
		leftovers.append(csv[2])	
		leftovers.append(csv[4])
		leftovers.append(csv[3])	
		leftovers.append(csv[5])
		points.append(float(csv[0]))
		points.append(float(csv[1]))
	count =+ 1

chs = cubic_hermite_spline()
points_arr = np.array(points).reshape(int(len(points)/2),2)
lefto_arr = np.array(leftovers).reshape(int(len(leftovers)/4),4)
for i in range(len(points_arr)-1):	
	if(int(i) == 0): 
		p1 = p0
		p2 = points_arr[i]
		p3 = points_arr[i+1]
	elif(int(i) == len(points_arr)):
		p1 = points_arr[i-1]
		p2 = points_arr[i]
		p3 = pn
	else:
		p1 = points_arr[i-1]
		p2 = points_arr[i]
		p3 = points_arr[i+1]

	t1 = tangent_cal(p1,p2,0.5)
	t2 = tangent_cal(p2,p3,0.5)
	steps = 25

	rte = chs.goto_wpt (p1,t1,p2,t2,steps)
	p = []
	for k in range(steps):
		p.append(rte[int(k)][0])
		p.append(rte[int(k)][1])
		p.append(lefto_arr[int(i)][0])
		p.append(lefto_arr[int(i)][1])
		p.append(lefto_arr[int(i)][2])
		p.append(lefto_arr[int(i)][3])
	write_to_file(p)

f.close()
ff.close()

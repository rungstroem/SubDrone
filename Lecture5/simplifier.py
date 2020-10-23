#!/usr/bin/env python3
f = open("convert_u/UTM_positions.csv", 'r');
ff = open("convert_u/UTM_simplified.csv", "a");
from math import pi, cos, acos, sin, atan, atan2, sqrt
import matplotlib.pyplot as plt

from rdp import rdp
import numpy as np

list1 = [[1,1], [2, 2], [3,3], [4,4]]
list2 =rdp(list1)

print(list1)
print(list2)
points = [];
leftovers = [];

for line in f:

	csv = line.split(",")
	
	points.append(float(csv[0]))
	points.append(float(csv[1]))
	leftovers.append(float(csv[2]))
	leftovers.append(csv[3])
	leftovers.append(float(csv[4]))
	leftovers.append(csv[5])

points_arr = np.array(points).reshape(int(len(points)/2),2)
leftovers_arr = np.array(leftovers).reshape(int(len(leftovers)/4),4)

test = rdp(points_arr, 0.5, algo="iter")

print(len(points_arr))
print(len(test))
print(leftovers_arr)
#print(test[0,1])
for i in range(len(test)):
	ff.write(str(test[i,0]))
	ff.write(",")
	ff.write(str(test[i,1]))
	ff.write(",")
	ff.write(str(leftovers_arr[i,0]))
	ff.write(",")
	ff.write(str(leftovers_arr[i,1]))
	ff.write(",")
	ff.write(str(leftovers_arr[i,2]))
	ff.write(",")
	ff.write(str(leftovers_arr[i,3]))


ff.close()
f.close()

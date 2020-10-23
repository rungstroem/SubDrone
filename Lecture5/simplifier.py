#!/usr/bin/env python3
f = open("convert_u/UTM_positions.csv", 'r');
ff = open("UTM_simplified.csv", "a");
from math import pi, cos, acos, sin, atan, atan2, sqrt
import matplotlib.pyplot as plt

from rdp import rdp
import numpy as np

list1 = [[1,1], [2, 2], [3,3], [4,4]]
list2 =rdp(list1)

print(list1)
print(list2)
points = [];

for line in f:

	csv = line.split(",")
	
	points.append(float(csv[0]))
	points.append(float(csv[1]))


points_arr = np.array(points).reshape(int(len(points)/2),2)


test = rdp(points_arr, 1, algo="iter")

print(points)	
print("\n")
print(len(points_arr))	
print(len(test))

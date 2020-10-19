#!/usr/bin/env python3
import matplotlib.pyplot as plt
from math import pi, cos, sin, acos, atan2, sqrt

f = open("UTM_positions.csv", 'r');
ff = open("UTM_positions_corrected.csv" ,'a');

dt = 1/100;	# Hz
velocity = 1.66 #m/s
magArray = [];
i = 0;
c_old = 0
c_new = 0
for line in f : 
	
	csv = line.split(",")

	x = float(csv[0])
	y = float(csv[1])
	z = float(csv[2])
	
	c_new = sqrt(x**2+y**2);
	if(i > 0 and (c_new - c_old) < velocity*dt):
		ff.write(str(x));
		ff.write(',');
		ff.write(str(y));
		ff.write(',');
		ff.write(str(z));
		ff.write('\n');
	elif(i == 0):
		ff.write(str(x));
		ff.write(',');
		ff.write(str(y));
		ff.write(',');
		ff.write(str(z));
		ff.write('\n');
	
	c_old = c_new
	i += 1;

f.close()	
ff.close()
#plt.plot(c)
#plt.show()	

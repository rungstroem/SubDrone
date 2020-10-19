#!/usr/bin/env python3

f = open("UTM_positions.csv","r")
ff = open("geo_pos.csv", "a")

from utm import utmconv
from math import pi, cos, acos, sin, atan2, sqrt

for line in f:
	
	csv = line.split(",")
	
	e2 = float(csv[0]) 
	n2 = float(csv[1]) 
	z = float(csv[2])
	hemisphere = str(csv[3])
	zone = float(csv[4])

	uc = utmconv()
	
	# convert back from UTM to geodetic
	(lat2, lon2) = uc.utm_to_geodetic (hemisphere, zone, e2, n2)
	
	ff.write(str(e2))
	ff.write(",")
	ff.write(str(n2))
	ff.write(",")
	ff.write(str(z))
	ff.write("\n")

ff.close()
f.close()

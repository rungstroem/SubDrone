#!/usr/bin/env python3

f = open("fixed_wing.csv","r")
ff = open("geo_pos_fixed.csv", "w")

from utm import utmconv
from math import pi, cos, acos, sin, atan2, sqrt

for line in f:
	
	csv = line.split(",")
	
	e2 = float(csv[0]) 
	n2 = float(csv[1]) 
	z = float(csv[2])
	hemisphere = str(csv[4])
	zone = float(csv[3])

	uc = utmconv()
	
	# convert back from UTM to geodetic
	(lat2, lon2) = uc.utm_to_geodetic (hemisphere, zone, e2, n2)
	
	ff.write(str(lat2))
	ff.write(",")
	ff.write(str(lon2))
	ff.write(",")
	ff.write(str(z))
	ff.write("\n")

ff.close()
f.close()

#590702.1639602276,6136753.502849699,19.817,N,32,U

#590702.1567078995,6136753.547228824,19.817,32,N,U


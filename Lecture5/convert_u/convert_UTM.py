f = open("positions.csv","r")
ff = open("UTM_positions.csv","a")
# import utmconv class
from utm import utmconv
from math import pi, cos, acos, sin, atan2, sqrt


for line in f:

	csv = line.split(",")
    
# geodetic reference coordinate
	lat1 =  float(csv[0])
	lon1 = float(csv[1])
	z = float(csv[2])
# Our geodetic reference coordinate


# instantiate utmconv class
	uc = utmconv()

	ff.write(str(lat1))
	ff.write(",")
	ff.write(str(lon1))
	ff.write(",")
	ff.write(str(z))
	ff.write("/n")

f.close()
ff.close()

#!/usr/bin/env python3
# import utmconv class
from utm import utmconv
from math import pi, cos, acos, sin, atan2, sqrt

# 1000m
m = 1000

# geodetic reference coordinate
lat1 =  55.47
lon1 = 010.33
# Our geodetic reference coordinate
lat1_1 = 68.530855
lon1_1 = -089.827172

print ('First position [deg]:')
print ('  latitude:  %.8f'  % (lat1))
print ('  longitude: %.8f'  % (lon1))

# instantiate utmconv class
uc = utmconv()
uc_1 = utmconv()

# convert from geodetic to UTM
(hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm (lat1,lon1)
print ('\nConverted from geodetic to UTM [m]')
print ('  %d %c %.5fe %.5fn' % (zone, letter, e1, n1))
(hemisphere_1, zone_1, letter_1, e1_1, n1_1) = uc_1.geodetic_to_utm (lat1_1,lon1_1)

# now generating the second UTM coordinate
e2 = e1 
n2 = n1 + m
e2_1 = e1_1 + m
n2_1 = n1_1

# convert back from UTM to geodetic
(lat2, lon2) = uc.utm_to_geodetic (hemisphere, zone, e2, n2)
print ('\nSecond position 100 meter East [deg]:')
print ('  latitude:  %.8f'  % (lat2))
print ('  longitude: %.8f'  % (lon2))
(lat2_1, lon2_1) = uc_1.utm_to_geodetic (hemisphere_1, zone_1, e2_1, n2_1)

# Distance via great circle distance formula:
R = 6371e3; ## metres
phi1 = lat1 * (pi/180); # φ, λ in radians
phi2 = lat2 * (pi/180);
dPhi = (lat2-lat1) * (pi/180);
dGamma = (lon2-lon1) * (pi/180);
a = sin(dPhi/2) * sin(dPhi/2) + cos(phi1) * cos(phi2) * sin(dGamma/2) * sin(dGamma/2);
c = 2 * atan2(sqrt(a), sqrt(1-a));
d = R * c; # in metres

print("Distance: ", d)
print("Difference: ", m-d)

# Distance via great circle distance formula:
R = 6371e3; ## metres
phi1_1 = lat1_1 * (pi/180); # φ, λ in radians
phi2_1 = lat2_1 * (pi/180);
dPhi_1 = (lat2_1-lat1_1) * (pi/180);
dGamma_1 = (lon2_1-lon1_1) * (pi/180);
a_1 = sin(dPhi_1/2) * sin(dPhi_1/2) + cos(phi1_1) * cos(phi2_1) * sin(dGamma_1/2) * sin(dGamma_1/2);
c_1 = 2 * atan2(sqrt(a_1), sqrt(1-a_1));
d_1 = R * c_1; # in metres

print("\nDistance: ", d_1)
print("Difference: ", m-d_1)

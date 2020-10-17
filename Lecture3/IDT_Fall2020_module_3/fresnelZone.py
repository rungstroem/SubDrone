#!/bin/env python3

from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse
import math

H = []
W = 403;
waveLength = (300*10**6)/(433*10**6);
dim1 = 201.5;
dim2 = 201.5;

for i in range(1, 5) :
	H.append( math.sqrt((i*waveLength*(dim1*dim2))/(dim1+dim2)) )

ellipse = []
for i in range(0, 4) :
	ellipse.append( Ellipse(xy=(0, 0), width=W, height=H[i], edgecolor='r', fc='None', lw=2)  )

plt.figure()
ax = plt.gca()
#ellipse433 = Ellipse(xy=(1, 1), width=403, height=8.33, edgecolor='r', fc='None', lw=2)
#ellipse2G4 = Ellipse(xy=(1, 1), width=403, height=3.55, edgecolor='g', fc='None', lw=2)

for i in range(0,4) :
	ax.add_patch(ellipse[i])

#ax.add_patch(ellipse433)
#ax.add_patch(ellipse2G4)
print(waveLength)
plt.title("First 4 Fresnel zones for 433MHz");
plt.xlabel("Fresnel zone length");
plt.ylabel("Fresnel zone heigth");
plt.plot()
plt.savefig('FresnelZones.png')
plt.show()

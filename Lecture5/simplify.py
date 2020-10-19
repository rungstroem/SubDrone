#!/usr/bin/env python3
f = open("UTM_positions.csv", 'r');
from math import pi, cos, acos, sin, atan, atan2, sqrt
import matplotlib.pyplot as plt

coordinatesX = [];
coordinatesY = [];
cX = [];
cY = [];
deg = 5;
def createCoordinates() :
	for line in f :
		getLine = line.split(',');
		coordinatesX.append(float(getLine[0]));
		coordinatesY.append(float(getLine[1]));
		cX.append(float(getLine[0]));
		cY.append(float(getLine[1]));


def minCoordinates() :
	theta1 = atan2(coordinatesY[0], coordinatesX[0]);
	offset = 1;
	for i in range(1, len(coordinatesX)):
		theta2 = atan2(coordinatesY[i], coordinatesX[i]);
		if((theta1 - theta2) < deg) :
			del cX[int(i)-offset];
			del cY[int(i)-offset];
			offset += 1;

def minCoord() :
	theta1 = atan(coordinatesY[0]/coordinatesX[0]);
	i = 0;
	while True :
		theta2 = atan(coordinatesY[i]/coordinatesX[i]);
		if( abs((theta1 - theta2)) < deg) :
			coordinatesX.pop();
			coordinatesY.pop();
		i += 1;
		if(i > len(coordinatesX)):
			break;
	return;

newX = [];
newY = [];

def remNPoint() :
	n = 100;
	index = len(coordinatesX)/n;
	for i in range(0, len(coordinatesX)) :
		if(i%n == 0) :
			newX.append(coordinatesX[i]);
			newY.append(coordinatesY[i]);

createCoordinates()
#minCoord()
remNPoint()

#createCoordinates();
fig = plt.figure()
ax = fig.add_subplot(2,1,1)
ax.scatter(newX, newY)
ax1 = fig.add_subplot(2,1,2)
ax1.scatter(cX, cY)
plt.show()

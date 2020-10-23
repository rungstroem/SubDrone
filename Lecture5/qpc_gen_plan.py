#!/usr/bin/python
f = open("convert_u/geo_pos.csv","r")
import json

plan = {}
geoFence = {}
plan['fileType'] = 'Plan'

geoFence['polygon'] = [] 
geoFence['version'] = 1 
plan['geoFence'] = geoFence

plan['groundStation'] = 'QGroundControl'

items = []
lat = 0
lon = 0
count = 0
for line in f:
	csv = line.split(",")

	if(count == 0):
		lon = float(csv[0])
		lat = float(csv[1])
		height = float(csv[2])	
	item = {}
	item['autoContinue'] = True
	item['command'] = 22
	item['doJumpId'] = 1
	item['frame'] = 3
	item['params'] = [0,0,0,0,float(csv[0]),float(csv[1]),50] # maybe change the last parameter to float(csv[3]) to get the height we got
	item['type'] = 'SimpleItem'
	items.append (item)
	count += 1 



mission = {}
mission['cruiseSpeed'] = 15
mission['firmwareType'] = 3
mission['hoverSpeed'] = 5
mission['items'] = items
mission['plannedHomePosition'] = [lon, lat, 50]
mission['vehicleType'] = 2
mission['version'] = 2
plan['mission'] = mission

rallyPoints = {}
rallyPoints['points'] = [] 
rallyPoints['version'] = 1 
plan['rallyPoints'] = rallyPoints

plan['version'] = 1

plan_json = json.dumps(plan, indent=4, sort_keys=True)

file = open('mission.plan','w') 
file.write (plan_json)
file.close()


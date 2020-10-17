#!/usr/bin/env python3
#/***************************************************************************
# MavLink LoRa node (ROS) upload mission example script
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
'''
This example script shows how to upload a single mission item to a drone.
This only works when used in My Master project with a companion computer onboard the drone
that handle single mission_items messages.

This does not work out-of-the-box with PX4 flight controller. Companion computer that handle these messages is needed

Revision
2018-06-13 FMA First published version
2018-03-14 FMA Cleaned scripts and made them better as examples
'''

# imports
import rospy
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_ack

# parameters
mavlink_lora_pub_topic = 'mavlink_interface/mission/upload_single'


def on_ack_received_callback(msg):
	# print ack
	print(msg.result_text)


# variables
target_sys = 0  # reset by first message
target_comp = 0
home_lat = 55.4720390
home_lon = 10.4147298

# launch node
rospy.init_node('mavlink_lora_mission_upload')

# pubs
mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_mission_item_int, queue_size=0)

# subs
mavlink_mission_ack_sub = rospy.Subscriber("mavlink_interface/mission/ack", mavlink_lora_mission_ack, on_ack_received_callback)

# wait until everything is running
rospy.sleep(1)

# # TAKEOFF waypoint
# way1 = mavlink_lora_mission_item_int()
# way1.target_system = 0
# way1.target_component = 0
# way1.seq = 2
# way1.frame = 6 #global pos, relative alt_int
# way1.command = 22
# way1.x = home_lat * 10000000
# way1.y = home_lon * 10000000
# way1.z = 20
# way1.param1 = 5
# way1.current = 1
# way1.autocontinue = 1

# WAYPOINT 1
way2 = mavlink_lora_mission_item_int()
way2.target_system = 0
way2.target_component = 0
way2.seq = 0
way2.frame = 6 #global pos, relative alt_int
way2.command = 16
way2.param1 = 0 # hold time
way2.param2 = 5 # acceptance radius in m
way2.param3 = 0 # pass though waypoint, no trajectory control
way2.x = 55.4720010 * 10000000
way2.y = 10.4164463 * 10000000
way2.z = 20
way2.autocontinue = 1

print("published single waypoint")
mavlink_msg_pub.publish(way2)

# loop until shutdown
while not (rospy.is_shutdown()):
	# do stuff
	rospy.sleep(5)

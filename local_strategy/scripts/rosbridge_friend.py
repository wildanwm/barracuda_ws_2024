#!/usr/bin/env python2
import rospy
from time import sleep
from enum import Enum
import math
# from math import sqrt, degrees, radians, sin, cos, atan2
from std_msgs.msg import Empty, Bool, Byte, Int8, Float64
from geometry_msgs.msg import Point, Pose2D, Twist, Quaternion

import websocket # type: ignore
import json
import threading

class RobotState(Enum):
	Stop = 0
	SetAbsPose = 1
	SetDribblePose = 2
	InitR = 3
	InitL = 4
	InitF = 5
	InitB = 6
	GotoBall = 7
	WaitBall = 8
	Distance2m = 9
	WaitFriend = 10

jersey_pose = Pose2D()
jersey_state = RobotState.Stop

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def JerseyPoseHandler(data):
	global jersey_pose
	jersey_pose = data

def JerseyStateHandler(data):
	global jersey_state
	jersey_state = RobotState(data.data)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class RobotPublisher:
	def __init__(self, ws_url, jersey_number):
		self.ws_url = ws_url
		self.jersey_number = jersey_number
		self.ws = None
		self.pose_data = {"x": 0.0, "y": 0.0, "theta": 0.0}
		self.state_data = RobotState.Stop
		self.running = True

	def start(self):
		while self.running and not rospy.is_shutdown():
			try:
				self.ws = websocket.WebSocketApp(self.ws_url,
												 on_open=self.on_open,
												 on_error=self.on_error,
												 on_close=self.on_close)
				self.ws.run_forever()
			except Exception as e:
				print(f"Connection error: {e}")
				print(f"Retrying in {self.retry_interval} seconds...")
				sleep(3)
			sleep(3)

	def on_open(self, ws):
		print("WebSocket connection opened on Robot")
		advertise_pose = {
			"op": "advertise",
			"topic": "/local_strategy/jersey"+str(self.jersey_number)+"/pose",
			"type": "geometry_msgs/Pose2D"
		}
		ws.send(json.dumps(advertise_pose))
		advertise_state = {
			"op": "advertise",
			"topic": "/local_strategy/jersey"+str(self.jersey_number)+"/state",
			"type": "std_msgs/Byte"
		}
		ws.send(json.dumps(advertise_state))
		publish_thread = threading.Thread(target=self.publish_continuous)
		publish_thread.start()

	def on_error(self, ws, error):
		print("WebSocket error on Robot:", error)

	def on_close(self, ws):
		print("WebSocket connection closed on Robot")
		self.running = False

	def publish_continuous(self):
		while self.running:
			publish_pose = {
				"op": "publish",
				"topic": "/local_strategy/jersey"+str(self.jersey_number)+"/pose",
				"msg": self.pose_data
			}
			self.ws.send(json.dumps(publish_pose))
			publish_state = {
				"op": "publish",
				"topic": "/local_strategy/jersey"+str(self.jersey_number)+"/state",
				"msg": {
					"data": self.state_data
				}
			}
			self.ws.send(json.dumps(publish_state))
			rospy.Rate(10).sleep()

	def update_pose_data(self, new_data):
		self.pose_data = new_data

	def update_state_data(self, new_data):
		self.state_data = new_data

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

if __name__ == '__main__':
	rospy.init_node('rosbridge_friend')

	ws_url = rospy.get_param("~ws_url", "ws://192.168.171.135:9090/")
	jersey_number = rospy.get_param("~jersey_number", "1")

	# rosbridge friend
	rospy.Subscriber('/local_strategy/jersey'+str(jersey_number)+'/pose', Pose2D, JerseyPoseHandler)
	rospy.Subscriber('/local_strategy/jersey'+str(jersey_number)+'/state', Byte, JerseyStateHandler)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

	sleep(7)

	robot_publisher = RobotPublisher(ws_url, jersey_number)
	threading.Thread(target=robot_publisher.start).start()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

	while not rospy.is_shutdown():
		robot_publisher.update_pose_data({"x": jersey_pose.x, "y": jersey_pose.y, "theta": jersey_pose.theta})
		robot_publisher.update_state_data(jersey_state.value)
		rospy.Rate(10).sleep()

	rospy.spin()

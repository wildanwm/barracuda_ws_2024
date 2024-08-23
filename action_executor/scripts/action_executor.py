#!/usr/bin/env python2
import rospy
from time import sleep
from enum import Enum
import math
# from math import sqrt, degrees, radians, sin, cos, atan2
from std_msgs.msg import Empty, Bool, Byte, Int8, Float64
from geometry_msgs.msg import Point, Pose2D, Twist, Quaternion

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

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

class PoseState(Enum):
	Disable = 0
	SetAbsPose = 1
	SetDribblePose = 2
	InitR = 3
	InitL = 4
	InitF = 5
	InitB = 6

class BallState(Enum):
	Disable = 0
	GotoBall = 1
	WaitBall = 2
	Distance2m = 3

command = RobotState.Stop
pose_data = Pose2D()
float_data = 0.0
pose_control_stats = PoseState.Disable
ball_control_stats = BallState.Disable

pose_target_feedback = Pose2D()
pose_mode_feedback = PoseState.Disable
ball_mode_feedback = BallState.Disable
robot_shoot_feedback = 0

robot_vel = Pose2D()
robot_vel.x = 0
robot_vel.y = 0
robot_vel.theta = 0

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# self
action_stats_pub = rospy.Publisher('/action_executor/action_stats', Int8, queue_size=10)
# pose control node
pose_target_pub = rospy.Publisher('/robot_control/pose_control/target', Pose2D, queue_size=10)
pose_mode_pub = rospy.Publisher('/robot_control/pose_control/mode', Int8, queue_size=10)
# ball control node
ball_mode_pub = rospy.Publisher('/robot_control/ball_control/mode', Int8, queue_size=10)
# serial node
robot_vel_target_pub = rospy.Publisher('/robot/vel_target', Pose2D, queue_size=10)

# send feedback
action_command_feedback = rospy.Publisher('/action_executor/command/feedback', Int8, queue_size=10)
action_pose_feedback = rospy.Publisher('/action_executor/pose_data/feedback', Pose2D, queue_size=10)
action_float_feedback = rospy.Publisher('/action_executor/float_data/feedback', Float64, queue_size=10)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def ActionCommandHandler(data):
	global command, pose_data, float_data, pose_control_stats, ball_control_stats
	command = RobotState(data.data)
	action_command_feedback.publish(data.data)
	rospy.loginfo("Action Executor (Command) : " + str(command))

	action_stats_pub.publish(command.value)

	if command == RobotState.Stop:                  # Stop
		pub_feedback.PoseMode(PoseState.Disable)
		pose_control_stats = PoseState.Disable
		pub_feedback.BallMode(BallState.Disable)
		ball_control_stats = BallState.Disable

	elif command == RobotState.SetAbsPose:          # SetAbsPose
		pub_feedback.PoseTarget(pose_data)
		pub_feedback.PoseMode(PoseState.SetAbsPose)
		pose_control_stats = PoseState.SetAbsPose
		pub_feedback.BallMode(BallState.Disable)
		ball_control_stats = BallState.Disable

	elif command == RobotState.SetDribblePose:		# SetDribblePose
		pub_feedback.PoseTarget(pose_data)
		pub_feedback.PoseMode(PoseState.SetDribblePose)
		pose_control_stats = PoseState.SetDribblePose
		pub_feedback.BallMode(BallState.Disable)
		ball_control_stats = BallState.Disable

	elif command == RobotState.InitR:              # InitR
		pub_feedback.PoseMode(PoseState.InitR)
		pose_control_stats = PoseState.InitR
		pub_feedback.BallMode(BallState.Disable)
		ball_control_stats = BallState.Disable

	elif command == RobotState.InitL:              # InitL
		pub_feedback.PoseMode(PoseState.InitL)
		pose_control_stats = PoseState.InitL
		pub_feedback.BallMode(BallState.Disable)
		ball_control_stats = BallState.Disable

	elif command == RobotState.InitF:              # InitF
		pub_feedback.PoseMode(PoseState.InitF)
		pose_control_stats = PoseState.InitF
		pub_feedback.BallMode(BallState.Disable)
		ball_control_stats = BallState.Disable

	elif command == RobotState.InitB:              # InitB
		pub_feedback.PoseMode(PoseState.InitB)
		pose_control_stats = PoseState.InitB
		pub_feedback.BallMode(BallState.Disable)
		ball_control_stats = BallState.Disable

	elif command == RobotState.GotoBall:            # GotoBall
		pub_feedback.PoseMode(PoseState.Disable)
		pose_control_stats = PoseState.Disable
		pub_feedback.BallMode(BallState.GotoBall)
		ball_control_stats = BallState.GotoBall

	elif command == RobotState.WaitBall:            # WaitBall
		pub_feedback.PoseMode(PoseState.Disable)
		pose_control_stats = PoseState.Disable
		pub_feedback.BallMode(BallState.WaitBall)
		ball_control_stats = BallState.WaitBall

	elif command == RobotState.Distance2m:            # FindBall
		pub_feedback.PoseMode(PoseState.Disable)
		pose_control_stats = PoseState.Disable
		pub_feedback.BallMode(BallState.Distance2m)
		ball_control_stats = BallState.Distance2m

	else:
		pass

#

def ActionPoseHandler(data):
	global pose_data
	pose_data = data
	action_pose_feedback.publish(data)
	rospy.loginfo("Action Executor (Pose) : " + str(pose_data))

def ActionFloatHandler(data):
	global float_data
	float_data = data.data
	action_float_feedback.publish(data.data)
	rospy.loginfo("Action Executor (Float) : " + str(float_data))

def PoseStatsHandler(data):
	global pose_control_stats
	pose_control_stats = PoseState(data.data)

def BallStatsHandler(data):
	global ball_control_stats
	ball_control_stats = BallState(data.data)

#

def PoseTargetFeedbackHandler(data):
	global pose_target_feedback
	pose_target_feedback = data

def PoseModeFeedbackHandler(data):
	global pose_mode_feedback
	pose_mode_feedback = PoseState(data.data)

def BallModeFeedbackHandler(data):
	global ball_mode_feedback
	ball_mode_feedback = BallState(data.data)

def RobotShootFeedbackHandler(data):
	global robot_shoot_feedback
	robot_shoot_feedback = data.data

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class ROSPubFeedback:
	def __init__(self):
		pass
	
	def PoseTarget(self, pose_target):
		pose_target_pub.publish(pose_target)
		sleep(0.05)
		while pose_target_feedback != pose_target:
			pose_target_pub.publish(pose_target)
			rospy.Rate(20).sleep()
	
	def PoseMode(self, pose_mode):
		pose_mode_pub.publish(pose_mode.value)
		sleep(0.05)
		while pose_mode_feedback != pose_mode:
			pose_mode_pub.publish(pose_mode.value)
			rospy.Rate(20).sleep()
	
	def BallMode(self, ball_mode):
		ball_mode_pub.publish(ball_mode.value)
		sleep(0.05)
		while ball_mode_feedback != ball_mode:
			ball_mode_pub.publish(ball_mode.value)
			rospy.Rate(20).sleep()

pub_feedback = ROSPubFeedback()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

if __name__ == '__main__':
	rospy.init_node('action_executor')

	# self
	rospy.Subscriber('/action_executor/command', Int8, ActionCommandHandler)
	rospy.Subscriber('/action_executor/pose_data', Pose2D, ActionPoseHandler)
	rospy.Subscriber('/action_executor/float_data', Float64, ActionFloatHandler)
	# robot control nodes
	rospy.Subscriber('/robot_control/pose_control/stats', Int8, PoseStatsHandler)
	rospy.Subscriber('/robot_control/ball_control/stats', Int8, BallStatsHandler)

	# get feedback
	rospy.Subscriber('/robot_control/pose_control/target/feedback', Pose2D, PoseTargetFeedbackHandler)
	rospy.Subscriber('/robot_control/pose_control/mode/feedback', Int8, PoseModeFeedbackHandler)
	rospy.Subscriber('/robot_control/ball_control/mode/feedback', Int8, BallModeFeedbackHandler)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

	while not rospy.is_shutdown():
		if command == RobotState.Stop:
			robot_vel_target_pub.publish(robot_vel)
		elif command == RobotState.WaitFriend:
			pass
		else:
			sleep(0.1)
			if pose_control_stats == PoseState.Disable and ball_control_stats == BallState.Disable:
				robot_vel_target_pub.publish(robot_vel)
				command = RobotState.Stop
				
		action_stats_pub.publish(command.value)
		rospy.Rate(50).sleep()

	rospy.spin()

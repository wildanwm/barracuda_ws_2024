#!/usr/bin/env python2
import rospy
from time import sleep
from enum import Enum
import math
# from math import sqrt, degrees, radians, sin, cos, atan2
from std_msgs.msg import Empty, Bool, Byte, Int8, Float64
from geometry_msgs.msg import Point, Pose2D, Twist, Quaternion

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class Enable(Enum):
	Stop = 0
	OnPlay = 1
	StartPosition = 2
	DropBall = 3
	KickOffHome = 4
	KickOffAway = 5
	GoalKickHome = 6
	GoalKickAway = 7
	FreeKickHome = 8
	FreeKickAway = 9
	CornerHome = 10
	CornerAway = 11
	PenaltyHome = 12
	PenaltyAway = 13
	ThrowInHome = 14
	ThrowInAway = 15
	Empty = 16
	# Additional
	ResetOdometry = 17
	KickBall = 18

class Pose():
	Initial 		= [-3750, -6250, 0]		# [-3750, -6250, 0]
	DropBall 		= [-2000, -1250, 1000]	# [-2000, -1250, 0]
	KickOffHome 	= [-1250, -1250, 1045]  # [-1250, -1250, 1045]
	KickOffAway 	= [-500, -2250, 1000]	# [-500, -2250, 1000]
	GoalKickHome 	= [-2000, -4750, 1000]	# [-2000, -4750, 0]
	GoalKickAway 	= [-2000, -750, 1000]	# [-2000, -750, 0]
	FreeKickHome	= [-2000, -1000, 1000]	# [-2000, -1000, 0]
	FreeKickAway	= [-2000, -3250, 1000]	# [-2000, -3250, 0]
	CornerHome 		= [-3500, 6250, -1135]	# [-3500, 6250, -1135]
	CornerAway 		= [-3500, -2750, 1180]	# [-3500, -2750, 180]
	PenaltyHome 	= [-1000, -1000, 1000]	# [-1000, -1000, 1000]
	PenaltyAway 	= [-3000, -3750, 1000]	# [-3000, -3750, 0]
	Defense			= [-1000, -3750, 1000]	# [-1000, -3750, 0]

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

enable = Enable.Stop
jersey1_pose = Pose2D()
jersey1_state = RobotState.Stop
robot_pose = Pose2D()
ball_reached = False
action_stats = RobotState.Stop
front_ball_detected = False
front_ball_point = Quaternion()
omni_ball_detected = False
omni_ball_point = Quaternion()
front_dummy_detected = False
front_dummy_point = Quaternion()
omni_dummy_detected = False
omni_dummy_point = Quaternion()

action_command_feedback = RobotState.Stop
action_pose_feedback = Pose2D()
action_float_feedback = 0.0
robot_initial_feedback = Pose2D()
robot_reset_feedback = False
robot_shoot_feedback = 0

arah_penalty = 0
ball_lost = False
ball_found = False
front_ball_ratio = -50
omni_ball_ratio = -50

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# rosbridge friend
jersey2_pose_pub = rospy.Publisher('/local_strategy/jersey2/pose', Pose2D, queue_size=10)
jersey2_state_pub = rospy.Publisher('/local_strategy/jersey2/state', Byte, queue_size=10)
# serial node
robot_initial_pub = rospy.Publisher('/robot/initial_pose', Pose2D, queue_size=10)
robot_reset_pub = rospy.Publisher('/robot/reset', Empty, queue_size=10)
robot_shoot_ball_pub = rospy.Publisher('/robot/shoot', Byte, queue_size=10)
# action executor node
action_command_pub = rospy.Publisher('/action_executor/command', Int8, queue_size=10)
action_pose_pub = rospy.Publisher('/action_executor/pose_data', Pose2D, queue_size=10)
action_float_pub = rospy.Publisher('/action_executor/float_data', Float64, queue_size=10)

# send feedback
enable_feedback = rospy.Publisher('/local_strategy/enable/feedback', Int8, queue_size=10)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def EnableHandler(data):
	global enable
	enable = Enable(data.data)
	enable_feedback.publish(data.data)
	rospy.loginfo("Local Strategy (Enable) : " + str(enable))

def Jersey1PoseHandler(data):
	global jersey1_pose
	jersey1_pose = data
	# rospy.loginfo(str(jersey1_pose.x) + "  " + str(jersey1_pose.y) + "  " + str(jersey1_pose.theta))

def Jersey1StateHandler(data):
	global jersey1_state
	jersey1_state = RobotState(data.data)
	# rospy.loginfo(RobotState(data.data))

def RobotPoseHandler(data):
	global robot_pose
	robot_pose = data

	jersey2_pose_pub.publish(data)
	# rospy.loginfo(str(robot_pose.x) + "  " + str(robot_pose.y) + "  " + str(robot_pose.theta))

def BallReachedHandler(data):
	global ball_reached
	ball_reached = data.data

def ActionStatsHandler(data):
	global action_stats
	action_stats = RobotState(data.data)
 
	jersey2_state_pub.publish(data.data)
	# rospy.loginfo(RobotState(data.data))

def FrontBallDetectedHandler(data):
	global front_ball_detected, front_ball_ratio
	front_ball_detected = data.data
	if front_ball_detected and front_ball_ratio < 50:
		front_ball_ratio += 1
	elif not front_ball_detected and front_ball_ratio > -50:
		front_ball_ratio -= 1

def FrontBallPointHandler(data):
	global front_ball_point, front_ball_detected
	if front_ball_detected:
		front_ball_point = data
	else:
		front_ball_point.x = 0
		front_ball_point.y = 0
		front_ball_point.z = 0
		front_ball_point.w = 0

def OmniBallDetectedHandler(data):
	global omni_ball_detected, omni_ball_ratio
	omni_ball_detected = data.data
	if omni_ball_detected and omni_ball_ratio < 50:
		omni_ball_ratio += 1
	elif not omni_ball_detected and omni_ball_ratio > -50:
		omni_ball_ratio -= 1

def OmniBallPointHandler(data):
	global omni_ball_point, omni_ball_detected
	if omni_ball_detected:
		omni_ball_point = data
	else:
		omni_ball_point.x = 0
		omni_ball_point.y = 0
		omni_ball_point.z = 0
		omni_ball_point.w = 0

def FrontDummyDetectedHandler(data):
	global front_dummy_detected
	front_dummy_detected = data.data

def FrontDummyPointHandler(data):
	global front_dummy_point, front_dummy_detected
	if front_dummy_detected:
		front_dummy_point = data
	else:
		front_dummy_point.x = 0
		front_dummy_point.y = 0
		front_dummy_point.z = 0
		front_dummy_point.w = 0

def OmniDummyDetectedHandler(data):
	global omni_dummy_detected
	omni_dummy_detected = data.data

def OmniDummyPointHandler(data):
	global omni_dummy_point, omni_dummy_detected
	if omni_dummy_detected:
		omni_dummy_point = data
	else:
		omni_dummy_point.x = 0
		omni_dummy_point.y = 0
		omni_dummy_point.z = 0
		omni_dummy_point.w = 0

#

def ActionCommandFeedbackHandler(data):
	global action_command_feedback
	action_command_feedback = RobotState(data.data)

def ActionPoseFeedbackHandler(data):
	global action_pose_feedback
	action_pose_feedback = data

def ActionFloatFeedbackHandler(data):
	global action_float_feedback
	action_float_feedback = data.data

def RobotInitialFeedbackHandler(data):
	global robot_initial_feedback
	robot_initial_feedback = data

def RobotResetFeedbackHandler(data):
	global robot_reset_feedback
	robot_reset_feedback = True
	sleep(0.1)
	robot_reset_feedback = False

def RobotShootFeedbackHandler(data):
	global robot_shoot_feedback
	robot_shoot_feedback = data.data

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class ROSPubFeedback:
	def __init__(self):
		pass
	
	def RobotInitial(self, initial_pose):
		robot_initial_pub.publish(initial_pose)
		sleep(0.05)
		while robot_initial_feedback != initial_pose:
			rospy.loginfo("Trying to Robot Initial Pose")
			robot_initial_pub.publish(initial_pose)
			rospy.Rate(50).sleep()

	def RobotReset(self, reset=True):
		robot_reset_pub.publish()
		sleep(0.05)
		while robot_reset_feedback != reset:
			rospy.loginfo("Trying to Robot Reset")
			robot_reset_pub.publish()
			rospy.Rate(50).sleep()
	
	def RobotShoot(self, power=20):
		robot_shoot_ball_pub.publish(power)
		sleep(0.05)
		while robot_shoot_feedback != power:
			rospy.loginfo("Trying to Robot Shoot")
			robot_shoot_ball_pub.publish(power)
			rospy.Rate(50).sleep()
	
	def ActionCommand(self, command):
		action_command_pub.publish(command.value)
		sleep(0.05)
		while action_command_feedback != command:
			action_command_pub.publish(command.value)
			rospy.Rate(50).sleep()
	
	def ActionPose(self, pose_data):
		action_pose_pub.publish(pose_data)
		sleep(0.05)
		while action_pose_feedback != pose_data:
			action_pose_pub.publish(pose_data)
			rospy.Rate(50).sleep()
	
	def ActionFloat(self, float_data):
		action_float_pub.publish(float_data)
		sleep(0.05)
		while action_float_feedback != float_data:
			action_float_pub.publish(float_data)
			rospy.Rate(50).sleep()

pub_feedback = ROSPubFeedback()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def InitReset(pose_array):
	rospy.loginfo("Robot Initial Pose & Reset")
	initial_pose = Pose2D()
	initial_pose.x = pose_array[0]
	initial_pose.y = pose_array[1]
	initial_pose.theta = pose_array[2]
	pub_feedback.RobotInitial(initial_pose)
	pub_feedback.RobotReset()
	rospy.loginfo("Robot Initial Pose & Reset Done")

def AutoInit(x=False, y=False, theta=False):
	ref_x = [-3800, 3800]
	dif_x = [abs(robot_pose.x - ref) for ref in ref_x]
	min_index_x = dif_x.index(min(dif_x))
	if x:
		val_x = ref_x[min_index_x]
	else:
		val_x = robot_pose.x

	ref_y = [-2800, -200, 200, 2800]					# [-5800, -200, 200, 5800]
	dif_y = [abs(robot_pose.y - ref) for ref in ref_y]
	min_index_y = dif_y.index(min(dif_y))
	if y:
		val_y = ref_y[min_index_y]
	else:
		val_y = robot_pose.y

	ref_theta = [-180, -90, 0, 90, 180]
	dif_theta = [abs(robot_pose.theta - ref) for ref in ref_theta]
	min_index_theta = dif_theta.index(min(dif_theta))
	if theta:
		val_theta = ref_theta[min_index_theta]
	else:
		val_theta = robot_pose.theta

	InitReset([val_x, val_y, val_theta])

def FullAutoInit():
	if robot_pose.x >= 0:
		if robot_pose.y > 0:
			SetAbsPose([2500, 1500, 0])
			InitR()
			SetAbsPose([2500, 1500, 0])
			InitB()
		elif robot_pose.y > -1500:			# -3000
			SetAbsPose([2500, -1500, 0])
			InitR()
			SetAbsPose([2500, -1500, 0])
			InitF()
		else:
			SetAbsPose([2500, -1500, 0])	# -4500
			InitR()
			SetAbsPose([2500, -1500, 0])	# -4500
			InitB()
	else:
		if robot_pose.y > 0:
			SetAbsPose([-2500, 1500, 0])
			InitL()
			SetAbsPose([-2500, 1500, 0])
			InitB()
		elif robot_pose.y > -1500:			# -3000
			SetAbsPose([-2500, -1500, 0])
			InitL()
			SetAbsPose([-2500, -1500, 0])
			InitF()
		else:
			SetAbsPose([-2500, -1500, 0])	# -4500
			InitL()
			SetAbsPose([-2500, -1500, 0])	# -4500
			InitB()
	# pass # BUKA INI SETELAH SELESAI TEST POSITIONING

def GetAngle(point_target):
	return float(math.degrees(math.atan2((point_target[0]-robot_pose.x), (point_target[1]-robot_pose.y))))

def KeeperPosition(y_keeper=6000):
	keeper_position = robot_pose.x + math.tan(math.radians(robot_pose.theta+front_dummy_point.w)) * (6000-robot_pose.y)
	# rospy.loginfo(keeper_position)
	return keeper_position

def WaitUntilDone(ball_lost_monitor=False, ball_found_monitor=False):
	global action_stats, enable, ball_lost, ball_found
	ball_lost = False
	ball_found = False
	sleep(0.1)
	while action_stats != RobotState.Stop and enable != Enable.Stop:
		if rospy.is_shutdown():
			break

		if ball_lost_monitor:
			if front_ball_ratio < 0 and omni_ball_ratio < 0:
				ball_lost = True
				action_stats = RobotState.Stop
				pub_feedback.ActionCommand(RobotState.Stop)
				break
			else:
				ball_lost = False

		if ball_found_monitor:
			if front_ball_ratio > 0 or omni_ball_ratio > 0:
				ball_found = True
				action_stats = RobotState.Stop
				pub_feedback.ActionCommand(RobotState.Stop)
				break
			else:
				ball_found = False
		rospy.Rate(50).sleep()

def WaitUntilBallReached():
	global ball_reached, enable
	sleep(0.1)
	while not ball_reached and enable != Enable.Stop:
		if rospy.is_shutdown():
			break
		rospy.Rate(50).sleep()

def WaitUntilBallMoved(time_out=7):
	global front_ball_point
	sleep(0.1)
	init_z = front_ball_point.z
	time_count = 0
	while abs(front_ball_point.z-init_z) < 0.05 and enable != Enable.Stop:
		if rospy.is_shutdown():
			break
		if time_count >= time_out:
			break
		time_count += 1 / 50
		rospy.Rate(50).sleep()

def WaitUntilFriendReady(time_out=30):
	global jersey1_state
	WaitFriend()
	time_count = 0
	while jersey1_state != RobotState.WaitBall and enable != Enable.Stop:
		if rospy.is_shutdown():
			break
		if time_count >= time_out:
			break
		time_count += 1 / 50
		rospy.Rate(50).sleep()

	sleep(0.25)
	SetAbsPose([robot_pose.x, robot_pose.y, GetAngle([jersey1_pose.x, jersey1_pose.y])])

#

def Stop():
	global action_stats
	action_stats = RobotState.Stop
	pub_feedback.ActionCommand(RobotState.Stop)				# Stop
	WaitUntilDone()

def SetAbsPose(pose_array, ball_found_monitor=False):
	global action_stats, enable
	action_stats = RobotState.SetAbsPose
	pose = Pose2D()
	pose.x = pose_array[0]
	pose.y = pose_array[1]
	pose.theta = pose_array[2]
	pub_feedback.ActionPose(pose)
	pub_feedback.ActionCommand(RobotState.SetAbsPose)		# SetAbsPose
	WaitUntilDone(ball_found_monitor=ball_found_monitor)

def SetDribblePose(pose_array):
	global action_stats
	action_stats = RobotState.SetDribblePose
	pose = Pose2D()
	pose.x = pose_array[0]
	pose.y = pose_array[1]
	pose.theta = pose_array[2]
	pub_feedback.ActionPose(pose)
	pub_feedback.ActionCommand(RobotState.SetDribblePose)	# SetAbsPose
	WaitUntilDone()

def InitR():
	global action_stats
	action_stats = RobotState.InitR
	pub_feedback.ActionCommand(RobotState.InitR)			# InitR
	WaitUntilDone()
	if 45 < abs(robot_pose.theta) < 135:
		AutoInit(x=False, y=True, theta=True)
	else:
		AutoInit(x=True, y=False, theta=True)

def InitL():
	global action_stats
	action_stats = RobotState.InitL
	pub_feedback.ActionCommand(RobotState.InitL)			# InitL
	WaitUntilDone()
	if 45 < abs(robot_pose.theta) < 135:
		AutoInit(x=False, y=True, theta=True)
	else:
		AutoInit(x=True, y=False, theta=True)

def InitF():
	global action_stats
	action_stats = RobotState.InitF
	pub_feedback.ActionCommand(RobotState.InitF)			# InitF
	WaitUntilDone()
	if 45 < abs(robot_pose.theta) < 135:
		AutoInit(x=True, y=False, theta=True)
	else:
		AutoInit(x=False, y=True, theta=True)

def InitB():
	global action_stats
	action_stats = RobotState.InitB
	pub_feedback.ActionCommand(RobotState.InitB)			# InitB
	WaitUntilDone()
	if 45 < abs(robot_pose.theta) < 135:
		AutoInit(x=True, y=False, theta=True)
	else:
		AutoInit(x=False, y=True, theta=True)

def GotoBall():
	global action_stats
	action_stats = RobotState.GotoBall
	pub_feedback.ActionCommand(RobotState.GotoBall)			# GoToBall
	WaitUntilDone(ball_lost_monitor=True)

def WaitBall(time_out=30):
	global action_stats, jersey1_state

	time_count = 0
	while jersey1_state != RobotState.WaitFriend and enable != Enable.Stop:
		if rospy.is_shutdown():
			break
		if time_count >= time_out:
			break
		time_count += 1/50
		rospy.Rate(50).sleep()

	sleep(0.25)
	SetAbsPose([robot_pose.x, robot_pose.y, GetAngle([jersey1_pose.x, jersey1_pose.y])])

	action_stats = RobotState.WaitBall
	pub_feedback.ActionCommand(RobotState.WaitBall)			# WaitBall
	WaitUntilDone()

def Distance2m():
	global action_stats
	action_stats = RobotState.Distance2m
	pub_feedback.ActionCommand(RobotState.Distance2m)		# Distance2m
	WaitUntilDone(ball_lost_monitor=True)

def ShootBall(power=20):
	global action_stats
	action_stats = RobotState.Stop
	pub_feedback.ActionCommand(RobotState.Stop)
	if  enable == Enable.Stop:
		power = 1
	sleep(1.5)
	pub_feedback.RobotShoot(power)							# ShootBall
	sleep(2)
 
def WaitFriend():
	global action_stats
	action_stats = RobotState.WaitFriend
	pub_feedback.ActionCommand(RobotState.WaitFriend)		# WaitFriend

#

def Play():
	global enable
	while enable == Enable.OnPlay:
		if jersey1_state != RobotState.GotoBall and jersey1_state != RobotState.SetDribblePose:
			GotoBall()
			if ball_lost:
				SetAbsPose([2000, -4000, 1000], ball_found_monitor=True)	# [3000, -4000, 1000]
				SetAbsPose([-3000, -2000, 1000], ball_found_monitor=True)	# [-3000, -2000, 1000]
				SetAbsPose([2000, 0, 1000], ball_found_monitor=True)		# [3000, 0, 1000]
				SetAbsPose([-3000, 2000, 1000], ball_found_monitor=True)	# [-3000, 2000, 1000]
				SetAbsPose([2000, 4000, 1000], ball_found_monitor=True)		# [3000, 4000, 1000]
			else:
				SetDribblePose([robot_pose.x, robot_pose.y, GetAngle([0, 6000])])	# [0, 6000]
				ShootBall(45)
				FullAutoInit()
		else:
			Distance2m()
			if ball_lost:
				SetAbsPose([2000, -4000, 1000], ball_found_monitor=True)	# [3000, -4000, 1000]
				SetAbsPose([-3000, -2000, 1000], ball_found_monitor=True)	# [-3000, -2000, 1000]
				SetAbsPose([2000, 0, 1000], ball_found_monitor=True)		# [3000, 0, 1000]
				SetAbsPose([-3000, 2000, 1000], ball_found_monitor=True)	# [-3000, 2000, 1000]
				SetAbsPose([2000, 4000, 1000], ball_found_monitor=True)		# [3000, 4000, 1000]

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

if __name__ == '__main__':
	rospy.init_node('nasional')

	# rosbridge websocket
	rospy.Subscriber('/local_strategy/enable', Int8, EnableHandler)
	# rosbridge friend
	rospy.Subscriber('/local_strategy/jersey1/pose', Pose2D, Jersey1PoseHandler)
	rospy.Subscriber('/local_strategy/jersey1/state', Byte, Jersey1StateHandler)
	# serial node
	rospy.Subscriber('/robot/pose', Pose2D, RobotPoseHandler)
	rospy.Subscriber('/robot/ball_reached', Bool, BallReachedHandler)
	# action executor node
	rospy.Subscriber('/action_executor/action_stats', Int8, ActionStatsHandler)
	# computer vision node
	rospy.Subscriber('/com_vision/front_cam/ball_detected', Bool, FrontBallDetectedHandler)
	rospy.Subscriber('/com_vision/front_cam/ball_point', Quaternion, FrontBallPointHandler)
	rospy.Subscriber('/com_vision/omni_cam/ball_detected', Bool, OmniBallDetectedHandler)
	rospy.Subscriber('/com_vision/omni_cam/ball_point', Quaternion, OmniBallPointHandler)
	rospy.Subscriber('/com_vision/front_cam/dummy_detected', Bool, FrontDummyDetectedHandler)
	rospy.Subscriber('/com_vision/front_cam/dummy_point', Quaternion, FrontDummyPointHandler)
	rospy.Subscriber('/com_vision/omni_cam/dummy_detected', Bool, OmniDummyDetectedHandler)
	rospy.Subscriber('/com_vision/omni_cam/dummy_point', Quaternion, OmniDummyPointHandler)
	# BUKA INI SETELAH SELESAI TEST POSITIONING

	# get feedback
	rospy.Subscriber('/action_executor/command/feedback', Int8, ActionCommandFeedbackHandler)
	rospy.Subscriber('/action_executor/pose_data/feedback', Pose2D, ActionPoseFeedbackHandler)
	rospy.Subscriber('/action_executor/float_data/feedback', Float64, ActionFloatFeedbackHandler)
	rospy.Subscriber('/robot/initial_pose/feedback', Pose2D, RobotInitialFeedbackHandler)
	rospy.Subscriber('/robot/reset/feedback', Empty, RobotResetFeedbackHandler)
	rospy.Subscriber('/robot/shoot', Byte, RobotShootFeedbackHandler)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

	sleep(3)
	InitReset(Pose.Initial)

	while not rospy.is_shutdown():
		if enable == Enable.Stop:
			Stop()
			enable = Enable.Empty

		elif enable == Enable.OnPlay:
			Play()

		elif enable == Enable.StartPosition:
			Distance2m()
			enable = Enable.Empty

		elif enable == Enable.DropBall:								# Drop Ball
			FullAutoInit()
			SetAbsPose(Pose.DropBall)
			if front_ball_detected:
				here = True
			else:
				SetAbsPose([-Pose.DropBall[0]-2250, robot_pose.y, 1085])
				here = False
			while enable == Enable.DropBall:
				pass
			if enable == Enable.OnPlay:
				if here:
					GotoBall()
					SetDribblePose([robot_pose.x, robot_pose.y, 85])
					ShootBall(20)
				else:
					WaitUntilBallMoved(3)
			Play()

		elif enable == Enable.KickOffHome:							# Kick Off Home
			FullAutoInit()
			SetAbsPose(Pose.KickOffHome)
			while enable == Enable.KickOffHome:
				pass
			if enable == Enable.OnPlay:
				WaitUntilBallMoved(3)
			Play()

		elif enable == Enable.KickOffAway:							# Kick Off Away
			FullAutoInit()
			SetAbsPose(Pose.KickOffAway)
			while enable == Enable.KickOffAway:
				pass
			if enable == Enable.OnPlay:
				WaitUntilBallMoved()
			Play()

		elif enable == Enable.GoalKickHome:							# Goal Kick Home
			FullAutoInit()
			SetAbsPose(Pose.GoalKickHome)
			if front_ball_detected:
				here = True
			else:
				SetAbsPose([-Pose.GoalKickHome[0]-2250, robot_pose.y, 1085])
				here = False
			while enable == Enable.GoalKickHome:
				pass
			if enable == Enable.OnPlay:
				if here:
					GotoBall()
					SetDribblePose([robot_pose.x, robot_pose.y, GetAngle([0, 6000])])
					ShootBall(95)
				else:
					WaitUntilBallMoved(3)
			Play()

		elif enable == Enable.GoalKickAway:							# Goal Kick Away
			FullAutoInit()
			SetAbsPose([Pose.GoalKickAway[0], Pose.GoalKickAway[1]+1000, Pose.GoalKickAway[2]])
			if front_ball_detected:
				SetAbsPose(Pose.GoalKickAway)
			else:
				SetAbsPose([-Pose.GoalKickAway[0]-1000, Pose.GoalKickAway[1], 1000])
			while enable == Enable.GoalKickAway:
				pass
			if enable == Enable.OnPlay:
				WaitUntilBallMoved()
			Play()

		elif enable == Enable.FreeKickHome:							# Free Kick Home
			FullAutoInit()
			SetAbsPose(Pose.FreeKickHome)
			if front_ball_detected:
				here = True
			else:
				SetAbsPose([-Pose.FreeKickHome[0]-2250, robot_pose.y, 1085])
				here = False
			while enable == Enable.FreeKickHome:
				pass
			if enable == Enable.OnPlay:
				if here:
					GotoBall()
					SetDribblePose([robot_pose.x, robot_pose.y, 85])
					ShootBall(20)
				else:
					WaitUntilBallMoved(3)
			Play()

		elif enable == Enable.FreeKickAway:							# Free Kick Away
			FullAutoInit()
			SetAbsPose(Pose.FreeKickHome)
			if front_ball_detected:
				SetAbsPose(Pose.FreeKickAway)
			else:
				SetAbsPose([-Pose.FreeKickAway[0]-1000, Pose.FreeKickAway[1], 1000])
			while enable == Enable.FreeKickAway:
				pass
			if enable == Enable.OnPlay:
				WaitUntilBallMoved()
			Play()

		elif enable == Enable.CornerHome:							# Corner Home
			FullAutoInit()
			SetAbsPose(Pose.CornerHome)
			if front_ball_detected:
				here = True
			else:
				SetAbsPose([0, 0, 1000])
				here = False
			while enable == Enable.CornerHome:
				pass
			if enable == Enable.OnPlay:
				if here:
					GotoBall()
					SetDribblePose([robot_pose.x, robot_pose.y, GetAngle([0, 5000])])
					ShootBall(45)
				else:
					pass
			Play()

		elif enable == Enable.CornerAway:							# Corner Away
			FullAutoInit()
			SetAbsPose([Pose.CornerAway[0], Pose.CornerAway[1]-1000, Pose.CornerAway[2]])
			if front_ball_detected:
				SetAbsPose(Pose.CornerAway)
			else:
				SetAbsPose([-Pose.CornerAway[0]-1000, Pose.CornerAway[1], 1135])
			while enable == Enable.CornerAway:
				pass
			if enable == Enable.OnPlay:
				WaitUntilBallMoved()
			Play()

		elif enable == Enable.PenaltyHome:
			FullAutoInit()
			SetAbsPose(Pose.PenaltyHome)
			while enable == Enable.PenaltyHome:
				pass
			if enable == Enable.OnPlay():
				WaitUntilBallMoved(3)
				GotoBall()
				if(arah_penalty % 3 == 1):
					SetDribblePose(robot_pose.x, robot_pose.y, GetAngle([-500, 6000]))
				elif(arah_penalty % 3 == 2):
					SetDribblePose(robot_pose.x, robot_pose.y, GetAngle([500, 6000]))
				else:
					SetDribblePose(robot_pose.x, robot_pose.y, GetAngle([0, 6000]))
			arah_penalty += 1
			ShootBall(45)
			sleep(1)
			ShootBall(45)
			sleep(1)
			ShootBall(45)
			enable = Enable.Empty

		elif enable == Enable.PenaltyAway:
			SetAbsPose(Pose.PenaltyAway)
			while enable == Enable.PenaltyAway:
				pass
			if enable == Enable.OnPlay:
				WaitUntilBallMoved()
			Play()

		elif enable == Enable.ThrowInHome:							# Throw In Home
			if (front_ball_detected or omni_ball_detected) and jersey1_state != RobotState.Distance2m:
				Distance2m()
				here = True
			else:
				SetAbsPose(Pose.Defense)
				here = False
			while enable == Enable.ThrowInHome:
				pass
			if enable == Enable.OnPlay:
				if here:
					GotoBall()
					SetDribblePose([robot_pose.x, robot_pose.y, GetAngle([0, 6000])])
					ShootBall(95)
				else:
					WaitUntilBallMoved(3)
			Play()

		elif enable == Enable.ThrowInAway:							# Throw In Away
			if (front_ball_detected or omni_ball_detected) and jersey1_state != RobotState.Distance2m:
				Distance2m()
				SetAbsPose([robot_pose.x, robot_pose.y-500, 1000])
				here = True
			else:
				SetAbsPose(Pose.Defense)
				here = False
			while enable == Enable.ThrowInAway:
				pass
			if enable == Enable.OnPlay:
				if here:
					WaitUntilBallMoved()
				else:
					WaitUntilBallMoved()
			Play()

		elif enable == Enable.ResetOdometry:
			InitReset(Pose.Initial)
			enable = Enable.Empty

		elif enable == Enable.KickBall:
			ShootBall(15)
			enable = Enable.Empty

		else:
			pass

		rospy.Rate(50).sleep()

	rospy.spin()

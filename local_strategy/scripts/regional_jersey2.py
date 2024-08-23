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
	Initial 		= [3700, 2000, -90]
	DropBall 		= [0, 0, 0]
	KickOffHome 	= [3000, 500, -90]
	KickOffAway 	= [1800, -200, -90]
	GoalKickHome 	= [0, 0, 0]
	GoalKickAway 	= [0, 0, 0]
	CornerHome 		= [0, 0, 0]
	CornerAway 		= [0, 0, 0]
	PenaltyHome 	= [0, 0, 0]
	PenaltyAway 	= [0, 0, 0]

class RobotState(Enum):
	Stop = 0
	SetAbsPose = 1
	SetDribblePose = 2
	InitRF = 3
	InitLF = 4
	InitF = 5
	InitFR = 6
	InitFL = 7
	GotoBall = 8
	WaitBall = 9
	FaceBall = 10
	WaitFriend = 11

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

dummy = [False, False, False, False, False, False, False, False, False, False, False, False]
arah_penalty = 0

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
	global front_ball_detected
	front_ball_detected = data.data

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
	global omni_ball_detected
	omni_ball_detected = data.data

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

def GetAngle(point_target):
	return float(math.degrees(math.atan2((point_target[0]-robot_pose.x), (point_target[1]-robot_pose.y))))

def KeeperPosition(y_keeper=6000):
	keeper_position = robot_pose.x + math.tan(math.radians(robot_pose.theta+front_dummy_point.w)) * (6000-robot_pose.y)
	# rospy.loginfo(keeper_position)
	return keeper_position

def WaitUntilDone():
	global action_stats, enable
	sleep(0.1)
	while action_stats != RobotState.Stop and enable != Enable.Stop:
		if rospy.is_shutdown():
			break
		rospy.Rate(50).sleep()

def WaitUntilBallReached():
	global ball_reached, enable
	sleep(0.1)
	while not ball_reached and enable != Enable.Stop:
		if rospy.is_shutdown():
			break
		rospy.Rate(50).sleep()

def WaitUntilBallMoved():
	global front_ball_point
	sleep(0.1)
	init_z = front_ball_point.z
	duration = 0
	while front_ball_point.z - init_z < 0.1 and enable != Enable.Stop:
		if rospy.is_shutdown():
			break
		if duration >= 3:
			break
		duration += 1 / 50
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
		time_count += 1/50
		rospy.Rate(50).sleep()

	sleep(0.25)
	SetAbsPose([robot_pose.x, robot_pose.y, GetAngle([jersey1_pose.x, jersey1_pose.y])])

#

def Stop():
	global action_stats
	action_stats = RobotState.Stop
	pub_feedback.ActionCommand(RobotState.Stop)				# Stop
	WaitUntilDone()

def SetAbsPose(pose_array):
	global action_stats, enable
	action_stats = RobotState.SetAbsPose
	pose = Pose2D()
	pose.x = pose_array[0]
	pose.y = pose_array[1]
	pose.theta = pose_array[2]
	pub_feedback.ActionPose(pose)
	pub_feedback.ActionCommand(RobotState.SetAbsPose)		# SetAbsPose
	WaitUntilDone()

	# For Trial
	# enable = Enable.Empty
	# while enable != Enable.OnPlay:
	# 	rospy.Rate(10)

def SetDribblePose(pose_array):
	global action_stats
	action_stats = RobotState.SetDribblePose
	pose = Pose2D()
	pose.x = pose_array[0]
	pose.y = pose_array[1]
	pose.theta = pose_array[2]
	pub_feedback.ActionPose(pose)
	pub_feedback.ActionCommand(RobotState.SetDribblePose)		# SetAbsPose
	WaitUntilDone()

def InitRF(pose_array):
	global action_stats
	action_stats = RobotState.InitRF
	pub_feedback.ActionCommand(RobotState.InitRF)			# InitRF
	WaitUntilDone()
	InitReset(pose_array)

def InitLF(pose_array):
	global action_stats
	action_stats = RobotState.InitLF
	pub_feedback.ActionCommand(RobotState.InitLF)			# InitLF
	WaitUntilDone()
	InitReset(pose_array)

def InitF(pose_array):
	global action_stats
	action_stats = RobotState.InitF
	pub_feedback.ActionCommand(RobotState.InitF)			# InitF
	WaitUntilDone()
	InitReset(pose_array)

def InitFR(pose_array):
	global action_stats
	action_stats = RobotState.InitFR
	pub_feedback.ActionCommand(RobotState.InitFR)			# InitFR
	WaitUntilDone()
	InitReset(pose_array)

def InitFL(pose_array):
	global action_stats
	action_stats = RobotState.InitFL
	pub_feedback.ActionCommand(RobotState.InitFL)			# InitFL
	WaitUntilDone()
	InitReset(pose_array)

def GotoBall():
	global action_stats
	action_stats = RobotState.GotoBall
	pub_feedback.ActionCommand(RobotState.GotoBall)			# GoToBall
	WaitUntilDone()

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

def FaceBall():
	global action_stats
	action_stats = RobotState.FaceBall
	pub_feedback.ActionCommand(RobotState.FaceBall)			# FaceBall
	WaitUntilDone()

def ShootBall(power=20):
	global action_stats
	action_stats = RobotState.Stop
	pub_feedback.ActionCommand(RobotState.Stop)
	if  enable == Enable.Stop:
		power = 1
	sleep(0.5)
	pub_feedback.RobotShoot(power)						# ShootBall
	sleep(0.5)
 
def WaitFriend():
	global action_stats
	action_stats = RobotState.WaitFriend
	pub_feedback.ActionCommand(RobotState.WaitFriend)		# WaitFriend

#

def Play():
	global enable
	while enable == Enable.OnPlay:
		pass

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

if __name__ == '__main__':
	rospy.init_node('regional')

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

		elif enable == Enable.KickOffHome:
			dummy = [False, False, False, False, False, False, False, False, False, False, False, False]
			InitReset(Pose.Initial)
			
			SetAbsPose([2250, 550, -90])
			InitLF([1750, -50, -90])
			SetAbsPose(Pose.KickOffHome)
			WaitBall()
			SetAbsPose([2250, 950, -90])
			InitFL([1750, 450, -90])
			WaitUntilFriendReady()
			ShootBall(24)

			SetAbsPose([2800, robot_pose.y, robot_pose.theta])
			SetAbsPose([3000, 3450, -88])	# [2800, 3500, -90]
			sleep(1)
			if front_dummy_detected and (front_dummy_point.y <= 0.68):
				if front_dummy_point.w > 0:
					dummy[6] = True
				else:
					dummy[5] = True
			if not dummy[6]:
				SetAbsPose([robot_pose.x, robot_pose.y+300, robot_pose.theta])
				SetAbsPose([2250, 3700, -90])	# [2250, 3750, -90]
				InitFR([1750, 4050, -90])
				sleep(1)
				if front_dummy_detected and (front_dummy_point.y <= 0.81) and (abs(front_dummy_point.w) <= 20) :
					dummy[8] = True
					if not dummy[5]:
						SetAbsPose([2800, 3000, -90])
					else:
						SetAbsPose([2800, 4000, -90])
				else:
					SetAbsPose([2800, 4000, -90])
					SetAbsPose([3000, 4800, -90])
			else:
				SetAbsPose([robot_pose.x, robot_pose.y-300, robot_pose.theta])
				SetAbsPose([2250, 3000, -90])	# [2250, 3000, -90]
				InitF([1750, robot_pose.y, -90])
				SetAbsPose([2800, 3000, -90])
			WaitBall()
			SetAbsPose([3250, 5000, -86])	# [-3250, 5050, 90]
			InitFR([2750, 5550, -90])
			SetAbsPose([2800, 5000, -90])
			WaitUntilFriendReady()
			ShootBall(25)

			# # #
			SetAbsPose([Pose.Initial[0] - 200, Pose.Initial[1], Pose.Initial[2]])
			Stop()
			enable = Enable.Empty

		elif enable == Enable.KickOffAway:
			dummy = [False, False, False, False, False, False, False, False, False, False, False, False]
			InitReset(Pose.Initial)
			
			SetAbsPose([2250, 550, -90])
			InitLF([1750, -50, -90])
			# SetAbsPose(Pose.KickOffHome)
			GotoBall()
			WaitUntilFriendReady()
			ShootBall(22)

			SetAbsPose([2250, 950, -90])
			InitFL([1750, 450, -90])
			SetAbsPose([2600, 1000, -90])
			WaitBall()
			SetAbsPose([2250, 950, -90])
			InitFL([1750, 450, -90])
			SetAbsPose([3000, robot_pose.y, robot_pose.theta])
			SetAbsPose([3400, 3600, -84])	# [3000, 3500, -90]
			sleep(0.1)
			if front_dummy_detected and (front_dummy_point.y <= 0.68):
				if front_dummy_point.w > 0:
					dummy[6] = True
				else:
					dummy[5] = True
			if not dummy[6]:
				SetAbsPose([robot_pose.x, robot_pose.y+300, robot_pose.theta])
				SetAbsPose([2750, 3900, -84])	# [2250, 3750, -90]
				InitFR([1750, 4050, -90])
				sleep(0.1)
				if front_dummy_detected and (front_dummy_point.y <= 0.81) and (abs(front_dummy_point.w) <= 20):
					dummy[8] = True
					if not dummy[5]:
						SetAbsPose([1750, 3000, -90])	# [1750, 3000, -90]
					else:
						SetAbsPose([2000, 4000, -90])	# [2000, 4000, -90]
			else:
				SetAbsPose([robot_pose.x, robot_pose.y-300, robot_pose.theta])
				SetAbsPose([2500, 3150, -86])	# [2250, 3000, -90]
				InitF([1750, robot_pose.y, -90])
			WaitUntilFriendReady()
			# SetAbsPose([robot_pose.x, robot_pose.y, GetAngle([jersey2_pose.x, jersey2_pose.y+200])])
			ShootBall(24)

			SetAbsPose([3050, robot_pose.y, robot_pose.theta])
			SetAbsPose([3250, 5350, -90])	# [-3250, 5050, 90]
			InitFR([2750, 5550, -90])
			SetAbsPose([2800, 5000, -90])
			WaitBall()
			if not dummy[6]:
				SetAbsPose([robot_pose.x, 3750, -90])
				SetAbsPose([2250, 3750, -90])
				InitFR([1750, 4050, -90])
				SetAbsPose([1500, 4000, GetAngle([1000, 6000])])	# [1500, 4000, GetAngle]
				SetAbsPose([1500, 4000, GetAngle([1000, 6000])])
			else:
				SetAbsPose([3000, robot_pose.y, -90])
				SetAbsPose([robot_pose.x, 2900, -90])
				SetAbsPose([2250, 2900, 92])	# [2250, 3000, -90]
				InitF([1750, robot_pose.y, -90])
				SetAbsPose([1400, 3000, GetAngle([0, 6000])])	# [-1400, 3000, GetAngle]
				SetAbsPose([1400, 3000, GetAngle([0, 6000])])
				while front_dummy_point.y < 0.75 and enable != Enable.Stop:
					rospy.loginfo("MAAJUUU NEEEHHH")
					SetAbsPose([robot_pose.x, robot_pose.y+200, GetAngle([0, 6000])])
			rospy.loginfo(KeeperPosition())
			if KeeperPosition() <= 175:
				SetAbsPose([robot_pose.x, robot_pose.y, GetAngle([500, 6000])])
			else:
				SetAbsPose([robot_pose.x, robot_pose.y, GetAngle([-500, 6000])])
			ShootBall(80)

			SetAbsPose([robot_pose.x, robot_pose.y-200, robot_pose.theta])
			SetAbsPose([3000, robot_pose.y, robot_pose.theta])
			# # #
			SetAbsPose([Pose.Initial[0]-200, Pose.Initial[1], Pose.Initial[2]])
			Stop()
			enable = Enable.Empty

		elif enable == Enable.CornerHome:
			pass

		elif enable == Enable.CornerAway:
			pass

		elif enable == Enable.PenaltyHome:
			SetAbsPose(Pose.PenaltyHome)
			while enable == Enable.PenaltyHome:
				pass
			GotoBall()
			# if(arah_penalty % 2 == 0):
			# 	SetTheta2Point([50, 600])
			# elif(arah_penalty % 3 == 0):
			# 	SetTheta2Point([-50, 600])
			# else:
			# 	SetTheta2Point([0, 600])
			arah_penalty += 1
			ShootBall()
			SetAbsPose(Pose.Initial)
			enable = Enable.Empty

		elif enable == Enable.PenaltyAway:
			pass

		else:
			pass

		rospy.Rate(50).sleep()

	rospy.spin()

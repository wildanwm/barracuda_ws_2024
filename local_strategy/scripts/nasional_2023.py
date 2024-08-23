#!/usr/bin/env python2
import rospy
from time import sleep
from math import atan2, degrees, sqrt
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Int8, Float64, Bool, Empty
from enum import Enum

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
	Initial = [-250, -150, 0]
	DropBall = [0, 0, 0]
	KickOffHome = [0, 0, 0]
	KickOff = [0, 0, 0]
	KickOffAway = [0, 0, 0]
	GoalKickHome = [0, 0, 0]
	GoalKickAway = [0, 0, 0]
	CornerHome = [-250, 100, 100]
	CornerAway = [0, 0, 0]
	PenaltyHome = [0, 0, 0]
	PenaltyAway = [0, 0, 0]

class RobotState(Enum):
	Stop = 0
	SetAbsPose = 1
	SetTheta = 2
	GotoBall = 3
	ShootBall = 4
	WaitBall = 5
	FaceBall = 6

stats = RobotState.Stop
lastStats = RobotState.Stop
currentPose = Pose2D()
ballReached = False
frontBallDetected = False
frontCurrent = Point() 
# dummy1 = 1
# dummy2 = 2

arahPenalty = 0

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# action executor node
command_pub = rospy.Publisher('/action_executor/command', Int8, queue_size=10)
pose_pub = rospy.Publisher('/action_executor/pose_data', Pose2D, queue_size=10)
float_pub = rospy.Publisher('/action_executor/float_data', Float64, queue_size=10)
# serial node
initial_pose_pub = rospy.Publisher('/robot/initial_pose', Pose2D, queue_size=10)
reset_pub = rospy.Publisher('/robot/reset', Empty, queue_size=10)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def enHandler(data):
	global enable
	enable = Enable(data.data)
	rospy.loginfo("Local Strategy : " + str(enable))

def actionStatHandler(data):
	global stats
	stats = RobotState(data.data)

def currentPoseHandler(data):
	global currentPose
	currentPose = data

def ballReachedHandler(data):
	global ballReached
	ballReached = data.data

def getKey(elem):
	return elem.z

def frontBallDetectedHandler(data):
	global frontBallDetected
	frontBallDetected = data.data

def frontBallHandler(data):
	global frontCurrent, frontBallDetected, X, Y
	frontCurrent = data

	if frontBallDetected:
		X = frontCurrent.x
		Y = frontCurrent.y

def resetHandler(data):
	global enable
	initial_pose = Pose2D()
	initial_pose.x = Pose.Initial[0]
	initial_pose.y = Pose.Initial[1]
	initial_pose.theta = Pose.Initial[2]

	prevEnable = enable
	enable = Enable.Stop
	sleep(0.5)
	initial_pose_pub.publish(initial_pose)
	enable = prevEnable
	sleep(0.1)
	rospy.loginfo("Local Strategy : Reset")

# def dummy1Handler(data):
#     global dummy1
#     dummy1 = data.data

# def dummy2Handler(data):
#     global dummy2
#     dummy2 = data.data

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def reset():
	rospy.loginfo("Reset")
	initial_pose = Pose2D()
	initial_pose.x = Pose.Initial[0]
	initial_pose.y = Pose.Initial[1]
	initial_pose.theta = Pose.Initial[2]

	while initial_pose != currentPose:
		initial_pose_pub.publish(initial_pose)
		sleep(0.1)
		reset_pub.publish()
		sleep(0.1)
	rospy.loginfo("Reset Done")

def publishPose(position, theta):
	pose = Pose2D()
	pose.x = position[0]
	pose.y = position[1]
	pose.theta = theta
	pose_pub.publish(pose)
	sleep(0.1)

def publishFloat(data):
	float_pub.publish(data)
	sleep(0.1)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def waitUntilDone():
	global stats, enable
	while stats != RobotState.Stop and enable != Enable.Stop:
		if rospy.is_shutdown():
			break
		rospy.Rate(20).sleep()
	sleep(0.1)

def waitUntilBallReached():
	global ballReached
	while not ballReached and enable != Enable.Stop:
		if rospy.is_shutdown():
			break
		rospy.Rate(20).sleep()
	sleep(0.1)

def waitUntilBallMoved():
	global X, Y
	initX = X
	initY = Y
	timer = 0
	while sqrt((X-initX)**2 + (Y-initY)**2) < 0.1 != Enable.Stop:
		if rospy.is_shutdown():
			break
		if timer >= 140:
			break
		timer += 1
		rospy.Rate(20).sleep()
	sleep(0.1)

def getAngle(robotPose, pointTarget):
	return float(degrees(atan2((pointTarget[0]-robotPose.x), (pointTarget[1]-robotPose.y))))

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def gotoPose(pose):
	global stats
	stats = RobotState.SetAbsPose
	publishPose(pose[:2], pose[2])
	command_pub.publish(RobotState.SetAbsPose.value)        # SetAbsPose
	waitUntilDone()

def gotoPosition(position):
	global stats
	stats = RobotState.SetAbsPose
	publishPose(position, currentPose.theta)
	command_pub.publish(RobotState.SetAbsPose.value)        # SetAbsPose
	waitUntilDone()

def setTheta(theta):
	global stats
	stats = RobotState.SetTheta
	publishFloat(theta)
	command_pub.publish(RobotState.SetTheta.value)
	waitUntilDone()

def setTheta2Point(point):
	global stats
	stats = RobotState.SetTheta
	publishFloat(getAngle(currentPose, point))
	command_pub.publish(RobotState.SetTheta.value)
	waitUntilDone()

def gotoBall():
	global stats
	stats = RobotState.GotoBall
	command_pub.publish(RobotState.GotoBall.value)
	waitUntilDone()

def WaitBall():
	global stats
	stats = RobotState.WaitBall
	command_pub.publish(RobotState.WaitBall.value)
	waitUntilDone()

def FaceBall():
	global stats
	stats = RobotState.FaceBall
	command_pub.publish(RobotState.FaceBall.value)
	waitUntilDone()

def ShootBall():
	if enable != Enable.Stop:
		global stats
		stats = RobotState.ShootBall
		command_pub.publish(RobotState.ShootBall.value)
		waitUntilDone()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def Play():
	while enable == Enable.OnPlay:
		gotoBall()
		gotoPose([currentPose.x-50, currentPose.y+50, -45])
		gotoPose([currentPose.x, currentPose.y+50, currentPose.theta])
		setTheta2Point([0, 625])
		ShootBall()

#############dribble#############
# def dribbleBall():
#     if ballReached:
#         if ([currentPose.x>-50 && currentPose.x<50, currentPose.y>250]):
#             ShootBall()
#         else :
#             stats = RobotState.dribbleBall
#             command_pub.publish(RobotState.dribbleBall.value)
#             gotoPosition([50, 300 ])
#             setTheta2Point([0,600])
#             ShootBall()
#             waitUntilDone()
#     else :
#         gotoBall()


if __name__ == '__main__':
	rospy.init_node('regional')

	enable = rospy.get_param("~local_en", False)

	# self
	rospy.Subscriber('/local_strategy/enable', Int8, enHandler)
	# action executor node
	rospy.Subscriber('/action_executor/action_stats', Int8, actionStatHandler)
	# serial node
	rospy.Subscriber('/robot/pose', Pose2D, currentPoseHandler)
	rospy.Subscriber('/robot/ball_reached', Bool, ballReachedHandler)
	rospy.Subscriber('/robot/reset', Empty, resetHandler)
	# computer vision node
	rospy.Subscriber('/com_vision/front_cam/detected', Bool, frontBallDetectedHandler)
	rospy.Subscriber('/com_vision/front_cam/circle_set', Point, frontBallHandler)
	# rospy.Subscriber('/local_strategy/dummy1', Int8, dummy1Handler)
	# rospy.Subscriber('/local_strategy/dummy2', Int8, dummy2Handler)

	command_pub.publish(RobotState.Stop.value)
	sleep(5)
	reset()

	while not rospy.is_shutdown():
		if enable == Enable.Stop:
			command_pub.publish(RobotState.Stop.value)
			rospy.loginfo("Stop") 
			enable = Enable.Empty
		
		elif enable == Enable.OnPlay:
			Play()
		
		elif enable == Enable.StartPosition:
			gotoPose(Pose.KickOffHome)
			enable = Enable.Empty

		elif enable == Enable.DropBall:
			# gotoPose([currentPose.x, currentPose.y-150, currentPose.theta])
			gotoPose(Pose.DropBall)
			while enable == Enable.DropBall:
				pass
			Play()

		elif enable == Enable.KickOffHome:
			gotoPose(Pose.KickOff)
			# gotoPose(Pose.KickOffHome)
			while enable == Enable.KickOffHome:
				pass
			# if enable == Enable.OnPlay:
			#     sleep(2)
			#     gotoPose([0, -400, 0])
			Play()

		elif enable == Enable.KickOffAway:
			gotoPose(Pose.KickOffAway)
			FaceBall()
			while enable == Enable.KickOffAway:
				pass
			if enable == Enable.OnPlay:
				waitUntilBallMoved()
			Play()

		elif enable == Enable.GoalKickHome:
			gotoPose(Pose.GoalKickHome)
			while enable == Enable.GoalKickHome:
				pass
			Play()

		elif enable == Enable.GoalKickAway:
			gotoPose(Pose.GoalKickAway)
			FaceBall()
			while enable == Enable.GoalKickAway:
				pass
			if enable == Enable.OnPlay:
				waitUntilBallMoved()
			Play()

		elif enable == Enable.FreeKickHome:
			enable = Enable.GoalKickHome

		elif enable == Enable.FreeKickAway:
			enable = Enable.GoalKickAway

		elif enable == Enable.CornerHome:
			gotoPose(Pose.CornerHome)
			# while enable == Enable.CornerHome:
			#     pass
			gotoBall()
			gotoPose(Pose.PenaltyHome)
			ShootBall()
			gotoPose(Pose.Initial)
			enable = Enable.Empty

		elif enable == Enable.CornerAway:
			gotoPose(Pose.CornerAway)
			FaceBall()
			while enable == Enable.CornerAway:
				pass
			if enable == Enable.OnPlay:
				waitUntilBallMoved()
			Play()

		elif enable == Enable.PenaltyHome:
			gotoPose(Pose.PenaltyHome)
			# while enable == Enable.PenaltyHome:
			#     pass
			gotoBall()
			if(arahPenalty % 2 == 0):
				setTheta2Point([50, 600])
			elif(arahPenalty % 3 == 0):
				setTheta2Point([-50, 600])
			else:
				setTheta2Point([0, 600])
			arahPenalty += 1
			ShootBall()
			gotoPose(Pose.Initial)
			enable = Enable.Empty

		elif enable == Enable.PenaltyAway:
			gotoPose(Pose.PenaltyAway)
			enable = Enable.Empty

		elif enable == Enable.ThrowInHome:
			enable = Enable.GoalKickHome

		elif enable == Enable.ThrowInAway:
			enable = Enable.GoalKickAway

		rospy.Rate(20).sleep()

	rospy.spin()

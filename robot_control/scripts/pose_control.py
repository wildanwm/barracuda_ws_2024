#!/usr/bin/env python2
import rospy
from time import sleep
from enum import Enum
import math
# from math import sqrt, degrees, radians, sin, cos, atan2
from std_msgs.msg import Empty, Bool, Byte, Int8, Float64
from geometry_msgs.msg import Point, Pose2D, Twist, Quaternion
# import threading

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class PoseState(Enum):
	Disable = 0
	SetAbsPose = 1
	SetDribblePose = 2
	InitR = 3
	InitL = 4
	InitF = 5
	InitB = 6

pose_target = Pose2D()
pose_mode = PoseState.Disable
robot_pose = Pose2D()
omni_init_v1 = 0.0
omni_init_v2 = 0.0
omni_init_h1 = 0.0
omni_init_h2 = 0.0
front_ball_detected = False
front_ball_point = Quaternion()
omni_ball_detected = False
omni_ball_point = Quaternion()

robot_vel = Pose2D()

init_step = 0
face_ball = False

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# self
pose_stats_pub = rospy.Publisher('/robot_control/pose_control/stats', Int8, queue_size=10)
# serial node
robot_vel_pub = rospy.Publisher('/robot/vel_target', Pose2D, queue_size=10)

# send feedback
pose_target_feedback = rospy.Publisher('/robot_control/pose_control/target/feedback', Pose2D, queue_size=10)
pose_mode_feedback = rospy.Publisher('/robot_control/pose_control/mode/feedback', Int8, queue_size=10)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def PoseTargetHandler(data):
	global pose_target, face_ball
	pose_target = data
	pose_target_feedback.publish(data)
	rospy.loginfo("Pose Control (Target) : " + str(pose_target))

	if abs(pose_target.theta) > 180:
		pose_target.theta = (abs(pose_target.theta)-1000) * sign(pose_target.theta)
		face_ball = True
	else:
		face_ball = False

def PoseModeHandler(data):
	global pose_mode, init_step
	pose_mode = PoseState(data.data)
	pose_mode_feedback.publish(data.data)
	rospy.loginfo("Pose Control (Mode) : " + str(pose_mode))

	pose_stats_pub.publish(pose_mode.value)
 
	init_step = 0

def RobotPoseHandler(data):
	global robot_pose
	robot_pose = data
	# rospy.loginfo(str(robot_pose.x) + "  " + str(robot_pose.y) + "  " + str(robot_pose.theta))

def OmniInitV1(data):
    global omni_init_v1
    omni_init_v1 = data.data

def OmniInitV2(data):
    global omni_init_v2
    omni_init_v2 = data.data

def OmniInitH1(data):
    global omni_init_h1
    omni_init_h1 = data.data

def OmniInitH2(data):
    global omni_init_h2
    omni_init_h2 = data.data

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
	# rospy.loginfo(str(front_ball_point.x) + "  " + str(front_ball_point.y) + "  " + str(front_ball_point.z) + "  " + str(front_ball_point.w))

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
	# rospy.loginfo(str(omni_ball_point.x) + "  " + str(omni_ball_point.y) + "  " + str(omni_ball_point.z) + "  " + str(omni_ball_point.w))

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def map(x, in_min, in_max, out_min, out_max):
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def sign(val):
	if val != 0:
		return val / abs(val)
	else:
		return 0

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def RobotVelPub(vx, vy, vtheta):
	global robot_vel, max_v, min_v, acc_v, max_w, min_w, acc_w

	if math.sqrt((vx**2) + (vy**2)) > max_v:                        # max_v max_w limit
		v_max_current_scale = max_v / math.sqrt((vx**2) + (vy**2))
		vx *= v_max_current_scale
		vy *= v_max_current_scale
	if abs(vtheta) > max_w:
		vtheta = max_w * sign(vtheta)

	robot_vel.x += (acc_v / 50) * sign(vx - robot_vel.x)            # acc_v acc_w application
	robot_vel.y += (acc_v / 50) * sign(vy - robot_vel.y)
	robot_vel.theta += (acc_w / 50) * sign(vtheta - robot_vel.theta)

	if abs(vx - robot_vel.x) <= (acc_v / 50):       				# avoid a cache value
		robot_vel.x = vx
	if abs(vy - robot_vel.y) <= (acc_v / 50):
		robot_vel.y = vy
	if abs(vtheta - robot_vel.theta) <= (acc_w / 50):
		robot_vel.theta = vtheta

	robot_vel_pub.publish(robot_vel)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

if __name__ == '__main__':
	rospy.init_node('pose_control')

	max_v = rospy.get_param("~max_v", 1500)                 # 1000 mm/s
	min_v = rospy.get_param("~min_v", 250)                  # mm/s
	acc_v = rospy.get_param("~acc_v", 1000)                 # 1500 mm/s^2
	max_w = rospy.get_param("~max_w", 90)                  	# deg/s
	min_w = rospy.get_param("~min_w", 30)                   # deg/s
	acc_w = rospy.get_param("~acc_w", 270)                  # 135 deg/s^2

	tolerance_xy = rospy.get_param("~tolerance_xy", 62.5)	# mm
	brake_xy = rospy.get_param("~brake_xy", 1250)           # 1125 mm
	if brake_xy < (max_v ** 2) / (2 * acc_v):
		brake_xy = (max_v ** 2) / (2 * acc_v)
	tolerance_theta = rospy.get_param("~tolerance_theta", 3)    # deg
	brake_theta = rospy.get_param("~brake_theta", 30)           # 15 deg
	if brake_theta < (max_w ** 2) / (2 * acc_w):
		brake_theta = (max_w ** 2) / (2 * acc_w)

	# self
	rospy.Subscriber('/robot_control/pose_control/target', Pose2D, PoseTargetHandler)
	rospy.Subscriber('/robot_control/pose_control/mode', Int8, PoseModeHandler)
	# serial node
	rospy.Subscriber('/robot/pose', Pose2D, RobotPoseHandler)
	# computer vision node
	rospy.Subscriber('/com_vision/omni_cam/init_v1', Float64, OmniInitV1)
	rospy.Subscriber('/com_vision/omni_cam/init_v2', Float64, OmniInitV2)
	rospy.Subscriber('/com_vision/omni_cam/init_h1', Float64, OmniInitH1)
	rospy.Subscriber('/com_vision/omni_cam/init_h2', Float64, OmniInitH2)
	rospy.Subscriber('/com_vision/front_cam/ball_detected', Bool, FrontBallDetectedHandler)
	rospy.Subscriber('/com_vision/front_cam/ball_point', Quaternion, FrontBallPointHandler)
	rospy.Subscriber('/com_vision/omni_cam/ball_detected', Bool, OmniBallDetectedHandler)
	rospy.Subscriber('/com_vision/omni_cam/ball_point', Quaternion, OmniBallPointHandler)
	# BUKA INI SETELAH SELESAI TEST POSITIONING

	while not rospy.is_shutdown():

		if pose_mode == PoseState.Disable:
			init_step = 0

		elif pose_mode == PoseState.SetAbsPose:
			if abs(pose_target.x - robot_pose.x) >= brake_xy:                       # vx, vy, and vtheta calculation with max limit, min limit, and stop tolerance
				vx = max_v * sign(pose_target.x - robot_pose.x)
			else:
				# if abs(pose_target.x - robot_pose.x) >= (min_v ** 2) / (2 * acc_v):
				if abs(pose_target.x - robot_pose.x) >= tolerance_xy:
					# vx = min_v * sign(pose_target.x - robot_pose.x)
					vx = ((max_v-min_v)*0.15+min_v) * sign(pose_target.x - robot_pose.x)
				else:
					vx = 0
					
			if abs(pose_target.y - robot_pose.y) >= brake_xy:
				vy = max_v * sign(pose_target.y - robot_pose.y)
			else:
				# if abs(pose_target.y - robot_pose.y) >= (min_v ** 2) / (2 * acc_v):
				if abs(pose_target.y - robot_pose.y) >= tolerance_xy:
					# vy = min_v * sign(pose_target.y - robot_pose.y)
					vy = ((max_v-min_v)*0.15+min_v) * sign(pose_target.y - robot_pose.y)
				else:
					vy = 0
			
			if math.sqrt((pose_target.x - robot_pose.x)**2 + (pose_target.y - robot_pose.y)**2) >= tolerance_xy:
				# if abs(pose_target.theta - robot_pose.theta) >= (min_w ** 2) / (2 * acc_w):
				if abs(pose_target.theta - robot_pose.theta) >= tolerance_theta:
					vtheta = min_w * sign(pose_target.theta - robot_pose.theta)
				else:
					vtheta = 0
			else:
				if abs(pose_target.theta - robot_pose.theta) >= brake_theta:
					vtheta = max_w * sign(pose_target.theta - robot_pose.theta)
				else:
					# if abs(pose_target.theta - robot_pose.theta) >= (min_w ** 2) / (2 * acc_w):
					if abs(pose_target.theta - robot_pose.theta) >= tolerance_theta:
						# vtheta = min_w * sign(pose_target.theta - robot_pose.theta)
						vtheta = ((max_w-min_w)*0.1+min_w) * sign(pose_target.theta - robot_pose.theta)
					else:
						vtheta = 0
					
			if face_ball and (front_ball_detected or omni_ball_detected):
				if math.sqrt((pose_target.x - robot_pose.x)**2 + (pose_target.y - robot_pose.y)**2) >= tolerance_xy:
					if front_ball_detected:
						if abs(front_ball_point.w) >= tolerance_theta:
							vtheta = front_ball_point.w * 1.0
					elif omni_ball_detected:
						vtheta = omni_ball_point.w * 1.0
				else:
					if front_ball_detected:
						if abs(front_ball_point.w) >= tolerance_theta:
							vtheta = front_ball_point.w * 0.7
					elif omni_ball_detected:
						vtheta = omni_ball_point.w * 0.7
			
			vx0 = math.cos(math.radians(robot_pose.theta)) * vx - math.sin(math.radians(robot_pose.theta)) * vy     # vx and vy calculation depend on robot pose theta
			vy0 = math.cos(math.radians(robot_pose.theta)) * vy + math.sin(math.radians(robot_pose.theta)) * vx
			
			RobotVelPub(vx0, vy0, vtheta)
			
			if abs(pose_target.x - robot_pose.x) <= tolerance_xy and abs(pose_target.y - robot_pose.y) <= tolerance_xy and (abs(pose_target.theta - robot_pose.theta) <= tolerance_theta or face_ball):
				RobotVelPub(0, 0, 0)
				pose_mode = PoseState.Disable
				pose_stats_pub.publish(pose_mode.value)
				rospy.loginfo("Done")

		elif pose_mode == PoseState.SetDribblePose:
			ball_pose = Pose2D()
			ball_pose.x = robot_pose.x + (math.sin(math.radians(robot_pose.theta)) * 250)
			ball_pose.y = robot_pose.y + (math.cos(math.radians(robot_pose.theta)) * 250)
			ball_pose.theta = robot_pose.theta
			ball_target = Pose2D()
			ball_target.x = pose_target.x + (math.sin(math.radians(pose_target.theta)) * 250)
			ball_target.y = pose_target.y + (math.cos(math.radians(pose_target.theta)) * 250)
			ball_target.theta = pose_target.theta

			vx = 0
			vy = 0
			vtheta = 0

			if abs(ball_target.x - ball_pose.x) >= brake_xy:                       # vx, vy, and vtheta calculation with max limit, min limit, and stop tolerance
				vx = max_v * sign(ball_target.x - ball_pose.x)
			else:
				if abs(ball_target.x - ball_pose.x) >= tolerance_xy:
					vx = min_v * sign(ball_target.x - ball_pose.x)
				else:
					vx = 0
					
			if abs(ball_target.y - ball_pose.y) >= brake_xy:
				vy = max_v * sign(ball_target.y - ball_pose.y)
			else:
				if abs(ball_target.y - ball_pose.y) >= tolerance_xy:
					vy = min_v * sign(ball_target.y - ball_pose.y)
				else:
					vy = 0
					
			if math.sqrt((ball_target.x - ball_pose.x)**2 + (ball_target.y - ball_pose.y)**2) >= brake_xy:
				ball_target.theta = math.degrees(math.atan2((ball_target.x-ball_pose.x), (ball_target.y-ball_pose.y)))
				if abs(ball_target.theta - ball_pose.theta) >= brake_theta:
					vtheta = max_w * sign(ball_target.theta - ball_pose.theta)
				else:
					if abs(ball_target.theta - ball_pose.theta) >= tolerance_theta:
						vtheta = min_w * sign(ball_target.theta - ball_pose.theta)
					else:
						vtheta = 0
			else:
				if abs(ball_target.theta - ball_pose.theta) >= brake_theta:
					vtheta = max_w * sign(ball_target.theta - ball_pose.theta)
				else:
					if abs(ball_target.theta - ball_pose.theta) >= tolerance_theta:
						vtheta = min_w * sign(ball_target.theta - ball_pose.theta)
					else:
						vtheta = 0
			
			vx0 = math.cos(math.radians(ball_pose.theta)) * vx - math.sin(math.radians(ball_pose.theta)) * vy     # vx and vy calculation depend on robot pose theta
			vy0 = math.cos(math.radians(ball_pose.theta)) * vy + math.sin(math.radians(ball_pose.theta)) * vx
			# print(str(vx)+"   "+str(vy)+"   "+str(vtheta))
			RobotVelPub(vx0, vy0, vtheta)
			
			if abs(ball_target.x - ball_pose.x) <= tolerance_xy and abs(ball_target.y - ball_pose.y) <= tolerance_xy and abs(ball_target.theta - ball_pose.theta) <= tolerance_theta:
				RobotVelPub(0, 0, 0)
				pose_mode = PoseState.Disable
				pose_stats_pub.publish(pose_mode.value)
				rospy.loginfo("Done")

		elif pose_mode == PoseState.InitR:
			if init_step == 0:
				vx = max_v * 0.75											# 0.75
				robot_vel.y = 0
				robot_vel.theta = 0
				RobotVelPub(vx, vy, vtheta)
				if omni_init_v1 != -0.001 or omni_init_v2 != -0.001:
					init_step = 1
			elif init_step == 1:
				# if omni_init_v1 != -0.001 and omni_init_v2 != -0.001:
				# 	robot_vel.theta = (omni_init_v1 - omni_init_v2) * max_w * 0.25	# 0.25
				# else:
				# 	robot_vel.theta = 0
				if omni_init_v1 == -0.001:
					omni_init_v1 = 1.0
				if omni_init_v2 == -0.001:
					omni_init_v2 = 1.0
				robot_vel.theta = (omni_init_v1 - omni_init_v2) * max_w * 0.25	# 0.25
				robot_vel.x = (min(omni_init_v1,omni_init_v2)-0.5) * max_v * 0.5	# 0.5
				robot_vel.y = 0
				if abs(robot_vel.x) <= 30.0 and abs(robot_vel.theta) <= 3.0:
					init_step = 2
			else:
				robot_vel.x = 0
				robot_vel.y = 0
				robot_vel.theta = 0
				robot_vel_pub.publish(robot_vel)
				init_step = 0
				pose_mode = PoseState.Disable
				pose_stats_pub.publish(pose_mode.value)
				rospy.loginfo("Done")
			robot_vel_pub.publish(robot_vel)

		elif pose_mode == PoseState.InitL:
			if init_step == 0:
				vx = -max_v * 0.75											# 0.75
				robot_vel.y = 0
				robot_vel.theta = 0
				RobotVelPub(vx, vy, vtheta)
				if omni_init_v1 != -0.001 or omni_init_v2 != -0.001:
					init_step = 1
			elif init_step == 1:
				# if omni_init_v1 != -0.001 and omni_init_v2 != -0.001:
				# 	robot_vel.theta = (omni_init_v1 - omni_init_v2) * max_w * 0.25	# 0.25
				# else:
				# 	robot_vel.theta = 0
				if omni_init_v1 == -0.001:
					omni_init_v1 = -1.0
				if omni_init_v2 == -0.001:
					omni_init_v2 = -1.0
				robot_vel.theta = (omni_init_v1 - omni_init_v2) * max_w * 0.25	# 0.25
				robot_vel.x = (max(omni_init_v1,omni_init_v2)+0.5) * max_v * 0.5	# 0.5
				robot_vel.y = 0
				if abs(robot_vel.x) <= 30.0 and abs(robot_vel.theta) <= 3.0:
					init_step = 2
			else:
				robot_vel.x = 0
				robot_vel.y = 0
				robot_vel.theta = 0
				robot_vel_pub.publish(robot_vel)
				init_step = 0
				pose_mode = PoseState.Disable
				pose_stats_pub.publish(pose_mode.value)
				rospy.loginfo("Done")
			robot_vel_pub.publish(robot_vel)

		elif pose_mode == PoseState.InitF:
			if init_step == 0:
				robot_vel.x = 0
				vy = max_v * 0.75											# 0.75
				robot_vel.theta = 0
				RobotVelPub(vx, vy, vtheta)
				if omni_init_h1 != -0.001 or omni_init_h2 != -0.001:
					init_step = 1
			elif init_step == 1:
				# if omni_init_h1 != -0.001 and omni_init_h2 != -0.001:
				# 	robot_vel.theta = (omni_init_h1 - omni_init_h2) * max_w * 0.25	# 0.25
				# else:
				# 	robot_vel.theta = 0
				if omni_init_h1 == -0.001:
					omni_init_h1 = -1.0
				if omni_init_h2 == -0.001:
					omni_init_h2 = -1.0
				robot_vel.theta = (omni_init_h1 - omni_init_h2) * max_w * 0.25	# 0.25
				robot_vel.x = 0
				robot_vel.y = -(max(omni_init_h1,omni_init_h2)+0.5) * max_v * 0.5	# 0.5
				if abs(robot_vel.y) <= 30.0 and abs(robot_vel.theta) <= 3.0:
					init_step = 2
			else:
				robot_vel.x = 0
				robot_vel.y = 0
				robot_vel.theta = 0
				robot_vel_pub.publish(robot_vel)
				init_step = 0
				pose_mode = PoseState.Disable
				pose_stats_pub.publish(pose_mode.value)
				rospy.loginfo("Done")
			robot_vel_pub.publish(robot_vel)

		elif pose_mode == PoseState.InitB:
			if init_step == 0:
				robot_vel.x = 0
				vy = -max_v * 0.75											# 0.75
				robot_vel.theta = 0
				RobotVelPub(vx, vy, vtheta)
				if omni_init_h1 != -0.001 or omni_init_h2 != -0.001:
					init_step = 1
			elif init_step == 1:
				# if omni_init_h1 != -0.001 and omni_init_h2 != -0.001:
				# 	robot_vel.theta = (omni_init_h1 - omni_init_h2) * max_w * 0.25	# 0.25
				# else:
				# 	robot_vel.theta = 0
				if omni_init_h1 == -0.001:
					omni_init_h1 = 1.0
				if omni_init_h2 == -0.001:
					omni_init_h2 = 1.0
				robot_vel.theta = (omni_init_h1 - omni_init_h2) * max_w * 0.25	# 0.25
				robot_vel.x = 0
				robot_vel.y = -(min(omni_init_h1,omni_init_h2)-0.5) * max_v * 0.5	# 0.5
				if abs(robot_vel.y) <= 30.0 and abs(robot_vel.theta) <= 3.0:
					init_step = 2
			else:
				robot_vel.x = 0
				robot_vel.y = 0
				robot_vel.theta = 0
				robot_vel_pub.publish(robot_vel)
				init_step = 0
				pose_mode = PoseState.Disable
				pose_stats_pub.publish(pose_mode.value)
				rospy.loginfo("Done")
			robot_vel_pub.publish(robot_vel)

		else:
			pass

		pose_stats_pub.publish(pose_mode.value)
		rospy.Rate(50).sleep()

	rospy.spin()

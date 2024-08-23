#!/usr/bin/env python2
import rospy
from time import sleep
from enum import Enum
import math
# from math import sqrt, degrees, radians, sin, cos, atan2
from std_msgs.msg import Empty, Bool, Byte, Int8, Float64
from geometry_msgs.msg import Point, Pose2D, Twist, Quaternion

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class BallState(Enum):
	Disable = 0
	GotoBall = 1
	WaitBall = 2
	Distance2m = 3

ball_mode = BallState.Disable
front_ball_detected = False
front_ball_point = Quaternion()
omni_ball_detected = False
omni_ball_point = Quaternion()
ball_reached = False
robot_pose = Pose2D()

robot_vel = Pose2D()

near_timer = 0
near_duration = 0

distance_step = 0

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# self
ball_stats_pub = rospy.Publisher('/robot_control/ball_control/stats', Int8, queue_size=10)
# serial node
robot_vel_pub = rospy.Publisher('/robot/vel_target', Pose2D, queue_size=10)

# send feedback
ball_mode_feedback = rospy.Publisher('/robot_control/ball_control/mode/feedback', Int8, queue_size=10)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def BallModeHandler(data):
	global ball_mode, distance_step
	ball_mode = BallState(data.data)
	ball_mode_feedback.publish(data.data)
	rospy.loginfo("Ball Control (Mode) : " + str(ball_mode))

	ball_stats_pub.publish(ball_mode.value)

	distance_step = 0

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

def BallReachedHandler(data):
	global ball_reached
	ball_reached = data.data

def RobotPoseHandler(data):
	global robot_pose
	robot_pose = data
	# rospy.loginfo(str(robot_pose.x) + "  " + str(robot_pose.y) + "  " + str(robot_pose.theta))

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def map(x, in_min, in_max, out_min, out_max):
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

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
	rospy.init_node('ball_control')

	max_v = rospy.get_param("~max_v", 1500)                  	# mm/s
	min_v = rospy.get_param("~min_v", 250)                  	# mm/s
	acc_v = rospy.get_param("~acc_v", 1000)                  	# mm/s^2
	max_w = rospy.get_param("~max_w", 90)                   	# deg/s
	min_w = rospy.get_param("~min_w", 30)                   	# deg/s
	acc_w = rospy.get_param("~acc_w", 270)                   	# deg/s^2

	tolerance_xy = rospy.get_param("~tolerance_xy", 30)     	# mm
	tolerance_theta = rospy.get_param("~tolerance_theta", 3)	# deg

	front_cam_2m = rospy.get_param("front_cam_2m", 0.80)
	front_cam_1m = rospy.get_param("front_cam_1m", 0.69)
	front_cam_05m = rospy.get_param("front_cam_05m", 0.53)
	omni_cam_2m = rospy.get_param("omni_cam_2m", 0.56)
	omni_cam_1m = rospy.get_param("omni_cam_1m", 0.46)
	omni_cam_05m = rospy.get_param("omni_cam_05m", 0.35)

	# self
	rospy.Subscriber('/robot_control/ball_control/mode', Int8, BallModeHandler)
	# computer vision node
	rospy.Subscriber('/com_vision/front_cam/ball_detected', Bool, FrontBallDetectedHandler)
	rospy.Subscriber('/com_vision/front_cam/ball_point', Quaternion, FrontBallPointHandler)
	rospy.Subscriber('/com_vision/omni_cam/ball_detected', Bool, OmniBallDetectedHandler)
	rospy.Subscriber('/com_vision/omni_cam/ball_point', Quaternion, OmniBallPointHandler)
	# BUKA INI SETELAH SELESAI TEST POSITIONING
	# serial node
	rospy.Subscriber('/robot/ball_reached', Bool, BallReachedHandler)
	rospy.Subscriber('/robot/pose', Pose2D, RobotPoseHandler)

	# try:
	#     rospy.wait_for_message('/com_vision/front_cam/ball_detector/ball_point', CircleSetStamped)
	# except:
	#     pass

	while not rospy.is_shutdown():

		if ball_mode == BallState.Disable:
			near_timer = 0
			near_duration = 0
			distance_step = 0

		elif ball_mode == BallState.GotoBall:
			vx = 0
			vy = 0
			vtheta = 0
			if front_ball_detected:
				smooth_catch_speed = clamp(map(front_ball_point.y, front_cam_05m/4, front_cam_1m, min_v, max_v), min_v, max_v) / max_v
				vy = map(abs(front_ball_point.w), 0, 90, max_v, min_v+(max_v-min_v)/2) * 2.0 * smooth_catch_speed		# 2.0
				if abs(front_ball_point.w) >= tolerance_theta:
					vtheta = front_ball_point.w * 1.4 * smooth_catch_speed													# 1.4
					vx = - math.radians(vtheta) * 355 * 0.25											# 355 = radius, 0.25
			elif omni_ball_detected:
				vtheta = omni_ball_point.w * 1.4														# 1.4
				vx = map(omni_ball_point.x, -omni_cam_2m, omni_cam_2m, -max_v, max_v) * 1.0				# 1.0
				vy = map(omni_ball_point.y, -omni_cam_2m, omni_cam_2m, -max_v, max_v) * 1.0				# 1.0
			else:
				pass

			# if front_ball_detected:
			# 	deg_ball_from_robot_post = front_ball_point.w + robot_pose.theta
			# elif omni_ball_detected:
			# 	deg_ball_from_robot_post = omni_ball_point.w + robot_pose.theta
			# else:
			# 	deg_ball_from_robot_post = 0
			# if abs(deg_ball_from_robot_post) > 90:
			# 	retreat_speed = map(abs(deg_ball_from_robot_post), 90, 180, min_v, max_v) * 2.0
			# 	vx = vx + math.sin(math.radians(robot_pose.theta)) * retreat_speed
			# 	vy = vy - math.cos(math.radians(robot_pose.theta)) * retreat_speed
			# else:
			# 	retreat_speed = 0

			RobotVelPub(vx, vy, vtheta)

			if ball_reached:
				robot_vel.x = 0
				robot_vel.y = 0
				robot_vel.theta = 0
				robot_vel_pub.publish(robot_vel)
				RobotVelPub(0, 0, 0)
				ball_mode = BallState.Disable
				ball_stats_pub.publish(ball_mode.value)
				rospy.loginfo("Done")

		elif ball_mode == BallState.WaitBall:
			vx = 0
			vy = 0
			vtheta = 0
			if front_ball_detected and 0.1 < front_ball_point.y <= front_cam_2m:
				vx = map(front_ball_point.x, -1, 1, -max_v, max_v) * 2.0			# 2.2
				if vx >= 0:
					vx = map(vx, 0, max_v, min_v, max_v)
				else:
					vx = map(vx, 0, -max_v, -min_v, -max_v)
				if front_ball_point.y <= front_cam_1m:
					vy = map(abs(front_ball_point.x), 0, 1, 0, -max_v) * 4.2		# 4.2
				else:
					vy = 0
				if front_ball_point.y <= front_cam_05m:
					near_timer += 1
			# elif omni_ball_detected and omni_ball_point.y <= ?:
			else:
				pass

			if near_timer >= 3:
				near_duration += 1 / 50												# 50 = rate
			if near_duration > 0.5:
				near_timer = 0
				near_duration = 0
				RobotVelPub(0, 0, 0)
				ball_mode = BallState.GotoBall
				ball_stats_pub.publish(ball_mode.value)
				rospy.loginfo("Miss")

			RobotVelPub(vx, vy, vtheta)

			if ball_reached:
				near_timer = 0
				near_duration = 0
				RobotVelPub(0, 0, 0)
				ball_mode = BallState.Disable
				ball_stats_pub.publish(ball_mode.value)
				rospy.loginfo("Done")

		elif ball_mode == BallState.Distance2m:
			vx = 0
			vy = 0
			vtheta = 0
			if distance_step == 0:
				if front_ball_detected:
					vy = map((front_ball_point.y - front_cam_1m), 0.05, (1 - front_cam_1m), min_v, max_v) * 2.0		# 2.0
					if abs(front_ball_point.w) >= tolerance_theta:
						vtheta = front_ball_point.w * 1.4															# 1.4
				elif omni_ball_detected:
					vtheta = omni_ball_point.w * 1.4																# 1.4
					vx = map(omni_ball_point.x, -omni_cam_2m, omni_cam_2m, -max_v, max_v) * 1.0						# 1.0
					vy = map(omni_ball_point.y, -omni_cam_2m, omni_cam_2m, -max_v, max_v) * 1.0						# 1.0
				if (abs(vtheta) <= tolerance_theta*4) and abs(vx) <= tolerance_xy*10 and abs(vy) <= tolerance_xy*10:
					distance_step = 1
			elif distance_step == 1:
				if front_ball_detected:
					vy = map((front_ball_point.y - front_cam_2m), 0.05, (1 - front_cam_2m), min_v, max_v) * 2.0		# 2.0
					angle_target = math.degrees(math.atan2(robot_pose.x-0.0, robot_pose.y+6000.0))
					vx = map(robot_pose.theta-angle_target, 0, 90, 0, max_v) * 1.0									# 1.0
					print(vx)
					if abs(front_ball_point.w) >= tolerance_theta:
						vtheta = front_ball_point.w * 1.4															# 1.4
				if (abs(vtheta) <= tolerance_theta*2) and abs(vx) <= tolerance_xy*5 and abs(vy) <= tolerance_xy*5:
					distance_step = 2

			RobotVelPub(vx, vy, vtheta)

			if distance_step >= 2:
				robot_vel.x = 0
				robot_vel.y = 0
				robot_vel.theta = 0
				robot_vel_pub.publish(robot_vel)
				distance_step = 0
				RobotVelPub(0, 0, 0)
				ball_mode = BallState.Disable
				ball_stats_pub.publish(ball_mode.value)
				rospy.loginfo("Done")

		else:
			pass

		ball_stats_pub.publish(ball_mode.value)
		rospy.Rate(50).sleep()

	rospy.spin()

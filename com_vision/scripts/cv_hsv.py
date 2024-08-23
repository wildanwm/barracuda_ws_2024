# !/usr/bin/env python2
import rospy
from time import sleep
from enum import Enum
from math import sqrt, degrees, radians, sin, cos, atan2
from std_msgs.msg import Empty, Bool, Byte, Int8, Float64
from geometry_msgs.msg import Point, Pose2D, Twist, Quaternion

import cv2 # type: ignore
import numpy as np # type: ignore
from threading import Thread
print(cv2.__version__)
# rospy.loginfo(cv2.__version__)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

circle_point = Quaternion()
circle_point.x = 0
circle_point.y = 0
circle_point.z = 0
circle_point.w = 0

dummy_point = Quaternion()
dummy_point.x = 0
dummy_point.y = 0
dummy_point.z = 0
dummy_point.w = 0

glitch_v1 = 10
glitch_v2 = 10
glitch_h1 = 10
glitch_h2 = 10
center_v1 = 0
center_v2 = 0
center_h1 = 0
center_h2 = 0

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# self
detect_pub = rospy.Publisher('ball_detected', Bool, queue_size=10)
point_pub = rospy.Publisher('ball_point', Quaternion, queue_size=10)
init_v1_pub = rospy.Publisher('init_v1', Float64, queue_size=10)
init_v2_pub = rospy.Publisher('init_v2', Float64, queue_size=10)
init_h1_pub = rospy.Publisher('init_h1', Float64, queue_size=10)
init_h2_pub = rospy.Publisher('init_h2', Float64, queue_size=10)
dummy_detected_pub = rospy.Publisher('dummy_detected', Bool, queue_size=10)
dummy_point_pub = rospy.Publisher('dummy_point', Quaternion, queue_size=10)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class VideoStream:
	def __init__(self, src=0, exposure_val=0):
		self.stream = cv2.VideoCapture(src)

		self.stream.set(cv2.CAP_PROP_FPS, 30)
		self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
		self.frame_width = int(self.stream.get(cv2.CAP_PROP_FRAME_WIDTH))
		self.frame_height = int(self.stream.get(cv2.CAP_PROP_FRAME_HEIGHT))
		rospy.loginfo((self.frame_width, self.frame_height))
		self.fps = self.stream.get(cv2.CAP_PROP_FPS)
		rospy.loginfo(self.fps)

		if self.stream.get(cv2.CAP_PROP_EXPOSURE) is not None:
			self.exposure_val = self.stream.get(cv2.CAP_PROP_EXPOSURE)
			self.exposure_val = exposure_val
			self.stream.set(cv2.CAP_PROP_EXPOSURE, self.exposure_val)
		if self.stream.get(cv2.CAP_PROP_AUTOFOCUS) is not None:
			self.stream.set(cv2.CAP_PROP_AUTOFOCUS, 0)

		(self.grabbed, self.frame) = self.stream.read()
		self.stopped = False

	def start(self):
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		while True:
			if self.stopped:
				return
			(self.grabbed, self.frame) = self.stream.read()

	def read(self):
		return self.frame

	def stop(self):
		self.stopped = True
		self.stream.release()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

if __name__ == '__main__':
	rospy.init_node('cv_hsv')

	cam_index = rospy.get_param("~cam_index", 0)
	exposure_val = rospy.get_param("~exposure_val", 0)

	vs = VideoStream(src=cam_index, exposure_val=exposure_val).start()

	frame_width = vs.frame_width
	frame_height = vs.frame_height

	flip_frame = rospy.get_param("~flip_frame", False)
	flip_code = rospy.get_param("~flip_code", 1)
	blur_val = rospy.get_param("~blur_val", 17)

	x_offset = rospy.get_param("~x_offset", 0)
	y_offset = rospy.get_param("~y_offset", 0)
	cam_center = (frame_width // 2 + x_offset, frame_height // 2 - y_offset)

	hue_min = rospy.get_param("~hue_min", 5)
	hue_max = rospy.get_param("~hue_max", 15)
	sat_min = rospy.get_param("~sat_min", 100)
	sat_max = rospy.get_param("~sat_max", 255)
	val_min = rospy.get_param("~val_min", 100)
	val_max = rospy.get_param("~val_max", 255)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

	init = rospy.get_param("~init", True)

	hue_min_white = rospy.get_param("~hue_min_white", 0)
	hue_max_white = rospy.get_param("~hue_max_white", 180)
	sat_min_white = rospy.get_param("~sat_min_white", 0)
	sat_max_white = rospy.get_param("~sat_max_white", 25)
	val_min_white = rospy.get_param("~val_min_white", 200)
	val_max_white = rospy.get_param("~val_max_white", 255)

	init_v1 = cam_center[1] - rospy.get_param("~init_v1", 65)		# 800mm from center of robot
	init_v2 = cam_center[1] - rospy.get_param("~init_v2", -65)		# 600mm from center of robot
	length_v = rospy.get_param("~length_v", 100)					# 100p
	init_h1 = cam_center[0] + rospy.get_param("~init_h1", 65)		# 200mm from center of robot
	init_h2 = cam_center[0] + rospy.get_param("~init_h2", -65)		# -200mm from center of robot
	length_h = rospy.get_param("~length_h", 100)					# 100p
	init_slope = rospy.get_param("~init_slope", 0)					# slope tolerance

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

	dummy = rospy.get_param("~dummy", True)

	hue_min_black = rospy.get_param("~hue_min_black", 5)
	hue_max_black = rospy.get_param("~hue_max_black", 190)
	sat_min_black = rospy.get_param("~sat_min_black", 0)
	sat_max_black = rospy.get_param("~sat_max_black", 190)
	val_min_black = rospy.get_param("~val_min_black", 40)
	val_max_black = rospy.get_param("~val_max_black", 80)

	crop_side = rospy.get_param("~crop_side", 100)
	crop_top = rospy.get_param("~crop_top", 50)
	crop_bottom = rospy.get_param("~crop_bottom", 150)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

	while not rospy.is_shutdown():
		frame = vs.read()
		if not vs.grabbed: break

		if flip_frame:
			frame = cv2.flip(frame, flip_code)
		frame_ui = cv2.GaussianBlur(frame, (1, 1), 0)
		blur = cv2.GaussianBlur(frame, (blur_val, blur_val), 0)
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, (hue_min, sat_min, val_min), (hue_max, sat_max, val_max))

		# ret2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		cv2.line(frame_ui, (cam_center[0] - 10, cam_center[1]), (cam_center[0] + 10, cam_center[1]), (255, 0, 0), 2)
		cv2.line(frame_ui, (cam_center[0], cam_center[1] - 10), (cam_center[0], cam_center[1] + 10), (255, 0, 0), 2)
		cv2.line(frame_ui, (0, cam_center[1]), (frame_width, cam_center[1]), (255, 0, 0), 1)
		cv2.line(frame_ui, (cam_center[0], 0), (cam_center[0], frame_height), (255, 0, 0), 1)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

		if hierarchy is not None:
			x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
			centroid = (x + w // 2, y + h // 2)

			# kalman = cv2.KalmanFilter(4, 2, 0)
			# kalman.transitionMatrix = np.array([[1, 0, 1, 0],
			# 									[0, 1, 0, 1],
			# 									[0, 0, 1, 0],
			# 									[0, 0, 0, 1]], np.float32)
			# kalman.measurementMatrix = np.array([[1, 0, 0, 0],
			# 									 [0, 1, 0, 0]], np.float32)
			# kalman.processNoiseCov = np.array([[1, 0, 0, 0],
			# 								   [0, 1, 0, 0],
			# 								   [0, 0, 1, 0],
			# 								   [0, 0, 0, 1]], np.float32) * 1e-5
			# kalman.measurementNoiseCov = np.array([[1, 0],
			# 									[0, 1]], np.float32) * 1e-4
			# kalman.predict()

			# measurement = np.array([centroid[0], centroid[1]], np.float32)
			# kalman.correct(measurement)
			# predicted_centroid = kalman.statePost

			ball_radius = max(w, h) // 2

			if ball_radius >= 3:
				detect_pub.publish(1)
				circle_point.x = (centroid[0] - cam_center[0]) / cam_center[0]
				circle_point.y = (cam_center[1] - centroid[1]) / cam_center[1]
				circle_point.z = sqrt(circle_point.x ** 2 + circle_point.y ** 2)
				circle_point.w = degrees(atan2(circle_point.x, circle_point.y))
				point_pub.publish(circle_point)

				# cv2.circle(frame_ui, (int(round(predicted_centroid[0, 0] * 10, 0)) + ball_radius, int(round(predicted_centroid[1, 0] * 10, 0)) + ball_radius//2), ball_radius, (0, 255, 0), 2)
				cv2.circle(frame_ui, centroid, ball_radius, (0, 255, 0), 2)
				cv2.line(frame_ui, (cam_center[0], cam_center[1]), (centroid[0], centroid[1]), (0, 255, 0), 2)

				# print(int(round(predicted_centroid[0, 0] * 10, 0)))
				# print(centroid)
				# print(x, y, w, h)

			else:
				detect_pub.publish(0)
				cv2.circle(frame_ui, centroid, ball_radius, (0, 255, 0), 2)

		else:
			detect_pub.publish(0)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

		if init:

			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			edges = cv2.Canny(gray, 50, 125, apertureSize=3)
			mask_white = cv2.inRange(hsv, (hue_min_white, sat_min_white, val_min_white), (hue_max_white, sat_max_white, val_max_white))
			kernel_dilated = np.ones((20,20),np.uint8)
			dilated_mask = cv2.dilate(mask_white, kernel_dilated, iterations=1)
			combined = cv2.bitwise_and(edges, dilated_mask)
			mask_combined = combined != 0
			frame_ui[mask_combined] = [0, 0, 0]

			# cv2.imshow("Edges", edges)
			# cv2.imshow("Dilated Mask", dilated_mask)
			# cv2.imshow("Combined", combined)

			# Vertical 1
			data_v1 = [combined[init_v1, cam_center[0]+init_slope+i] for i in range(-length_v//2, length_v//2+1)]
			indices_v1 = [i for i, value in enumerate(data_v1) if value]
			if indices_v1:
				center_v1 = (max(indices_v1)-min(indices_v1))//2+min(indices_v1) - length_v//2
				init_v1_pub.publish(center_v1 / (length_v//2))
				cv2.rectangle(frame_ui, (cam_center[0]+init_slope-length_v//2, init_v1-2), (cam_center[0]+init_slope+length_v//2, init_v1+2), (0, 255, 0), 1)
				cv2.line(frame_ui, (cam_center[0]+init_slope+center_v1, init_v1-8), (cam_center[0]+init_slope+center_v1, init_v1+8), (0, 0, 0), 2)
				glitch_v1 = 0
			else:
				if glitch_v1 >= 10:
					init_v1_pub.publish(-0.001)
				else:
					init_v1_pub.publish(center_v1 / (length_v//2))
					glitch_v1 += 1
				cv2.rectangle(frame_ui, (cam_center[0]+init_slope-length_v//2, init_v1-2), (cam_center[0]+init_slope+length_v//2, init_v1+2), (255, 0, 0), 1)

			# Vertical 2
			data_v2 = [combined[init_v2, cam_center[0]-init_slope+i] for i in range(-length_v//2, length_v//2+1)]
			indices_v2 = [i for i, value in enumerate(data_v2) if value]
			if indices_v2:
				center_v2 = (max(indices_v2)-min(indices_v2))//2+min(indices_v2) - length_v//2
				init_v2_pub.publish(center_v2 / (length_v//2))
				cv2.rectangle(frame_ui, (cam_center[0]-init_slope-length_v//2, init_v2-2), (cam_center[0]-init_slope+length_v//2, init_v2+2), (0, 255, 0), 1)
				cv2.line(frame_ui, (cam_center[0]-init_slope+center_v2, init_v2-8), (cam_center[0]-init_slope+center_v2, init_v2+8), (0, 0, 0), 2)
				glitch_v2 = 0
			else:
				if glitch_v2 >= 10:
					init_v2_pub.publish(-0.001)
				else:
					init_v2_pub.publish(center_v2 / (length_v//2))
					glitch_v2 += 1
				cv2.rectangle(frame_ui, (cam_center[0]-init_slope-length_v//2, init_v2-2), (cam_center[0]-init_slope+length_v//2, init_v2+2), (255, 0, 0), 1)

			init_h_center = (init_v1 - init_v2) // 2 + init_v2

			# Horizontal 1
			data_h1 = [combined[init_h_center+init_slope+i, init_h1] for i in range(-length_v//2, length_v//2+1)]
			indices_h1 = [i for i, value in enumerate(data_h1) if value]
			if indices_h1:
				center_h1 = (max(indices_h1)-min(indices_h1))//2+min(indices_h1) - length_v//2
				init_h1_pub.publish(center_h1 / (length_v//2))
				cv2.rectangle(frame_ui, (init_h1-2, init_h_center+init_slope-length_v//2), (init_h1+2, init_h_center+init_slope+length_v//2), (0, 255, 0), 1)
				cv2.line(frame_ui, (init_h1-8, init_h_center+init_slope+center_h1), (init_h1+8, init_h_center+init_slope+center_h1), (0, 0, 0), 2)
				glitch_h1 = 0
			else:
				if glitch_h1 >= 10:
					init_h1_pub.publish(-0.001)
				else:
					init_h1_pub.publish(center_h1 / (length_v//2))
					glitch_h1 += 1
				cv2.rectangle(frame_ui, (init_h1-2, init_h_center+init_slope-length_v//2), (init_h1+2, init_h_center+init_slope+length_v//2), (255, 0, 0), 1)

			# Horizontal 2
			data_h2 = [combined[init_h_center-init_slope+i, init_h2] for i in range(-length_v//2, length_v//2+1)]
			indices_h2 = [i for i, value in enumerate(data_h2) if value]
			if indices_h2:
				center_h2 = (max(indices_h2)-min(indices_h2))//2+min(indices_h2) - length_v//2
				init_h2_pub.publish(center_h2 / (length_v//2))
				cv2.rectangle(frame_ui, (init_h2-2, init_h_center-init_slope-length_v//2), (init_h2+2, init_h_center-init_slope+length_v//2), (0, 255, 0), 1)
				cv2.line(frame_ui, (init_h2-8, init_h_center-init_slope+center_h2), (init_h2+8, init_h_center-init_slope+center_h2), (0, 0, 0), 2)
				glitch_h2 = 0
			else:
				if glitch_h2 >= 10:
					init_h2_pub.publish(-0.001)
				else:
					init_h2_pub.publish(center_h2 / (length_v//2))
					glitch_h2 += 1
				cv2.rectangle(frame_ui, (init_h2-2, init_h_center-init_slope-length_v//2), (init_h2+2, init_h_center-init_slope+length_v//2), (255, 0, 0), 1)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

		if dummy:
			cv2.rectangle(frame_ui, (crop_side+x_offset, crop_top), (frame_width-crop_side, frame_height-crop_bottom), (255, 0, 0), 1)
			cropped_hsv = hsv[crop_top:frame_height-crop_bottom, crop_side+x_offset:frame_width-crop_side]
			mask_black = cv2.inRange(cropped_hsv, (hue_min_black, sat_min_black, val_min_black), (hue_max_black, sat_max_black, val_max_black))
			# cv2.imshow("Mask Black", mask_black)
			# ret2, contours, hierarchy = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			contours, hierarchy = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

			if hierarchy is not None:
				x_cropped, y_cropped, w_dummy, h_dummy = cv2.boundingRect(max(contours, key=cv2.contourArea))
				rectangle_ratio = float(w_dummy)/h_dummy
				rectangle_area = float(w_dummy)*h_dummy
				if 0.25 <= rectangle_ratio <= 4.0 and rectangle_area >= 650:
					x_dummy = x_cropped + crop_side + x_offset
					y_dummy = y_cropped + crop_top
					centroid_dummy = (x_dummy + w_dummy // 2, y_dummy + h_dummy // 2)
					cv2.rectangle(frame_ui, (x_dummy, y_dummy), (x_dummy + w_dummy, y_dummy + h_dummy), (0, 0, 255), 2)
					cv2.line(frame_ui, (cam_center[0], cam_center[1]), (centroid_dummy[0], centroid_dummy[1]), (0, 0, 255), 2)

					dummy_detected_pub.publish(1)
					dummy_point.x = (centroid_dummy[0] - cam_center[0]) / cam_center[0]
					dummy_point.y = (cam_center[1] - centroid_dummy[1]) / cam_center[1]
					dummy_point.z = sqrt(dummy_point.x ** 2 + dummy_point.y ** 2)
					dummy_point.w = degrees(atan2(dummy_point.x, dummy_point.y))
					dummy_point_pub.publish(dummy_point)
				else:
					dummy_detected_pub.publish(0)
			else:
				dummy_detected_pub.publish(0)

			cv2.putText(frame_ui, f'x : {dummy_point.x:.2f}', (160, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
			cv2.putText(frame_ui, f'y : {dummy_point.y:.2f}', (240, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
			cv2.putText(frame_ui, f'z : {dummy_point.z:.2f}', (320, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
			cv2.putText(frame_ui, f'w : {dummy_point.w:.2f}', (400, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

		cv2.namedWindow("Frame " + str(cam_index), cv2.WINDOW_GUI_NORMAL)
		cv2.putText(frame_ui, f'x : {circle_point.x:.2f}', (160, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
		cv2.putText(frame_ui, f'y : {circle_point.y:.2f}', (240, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
		cv2.putText(frame_ui, f'z : {circle_point.z:.2f}', (320, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
		cv2.putText(frame_ui, f'w : {circle_point.w:.2f}', (400, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
		cv2.resizeWindow("Frame " + str(cam_index), 512, (512 * frame_height // frame_width))
		if cam_index == 0:
			cv2.moveWindow("Frame " + str(cam_index), 0, 0)
		else:
			cv2.moveWindow("Frame " + str(cam_index), 512, 0)
		cv2.imshow("Frame " + str(cam_index), frame_ui)
		# cv2.imshow("HSV", hsv)
		# cv2.imshow("Mask", mask)

		rospy.Rate(50).sleep()
		if cv2.waitKey(1) & 0xFF == ord('q'): break

	vs.stop()
	cv2.destroyAllWindows()
	rospy.spin()

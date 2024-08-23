# !/usr/bin/env python2
import rospy
from time import sleep

import cv2
import numpy as np
print(cv2.__version__)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

if __name__ == '__main__':
	rospy.init_node('hsv_calibrate_ball')

	cam_index = rospy.get_param("~cam_index", 0)
	cap = cv2.VideoCapture(cam_index)
	cap.set(cv2.CAP_PROP_FPS, 30)
	cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
	frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
	frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
	rospy.loginfo((frame_width, frame_height))
	fps = cap.get(cv2.CAP_PROP_FPS)
	rospy.loginfo(fps)

	flip_frame = rospy.get_param("~flip_frame", False)
	flip_code = rospy.get_param("~flip_code", 1)
	if cap.get(cv2.CAP_PROP_EXPOSURE) is not None:
		exposure_val = cap.get(cv2.CAP_PROP_EXPOSURE)
		exposure_val = rospy.get_param("~exposure_val", exposure_val)
		cap.set(cv2.CAP_PROP_EXPOSURE, exposure_val)
	blur_val = rospy.get_param("~blur_val", 17)
	if cap.get(cv2.CAP_PROP_AUTOFOCUS) is not None:
		cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

	hue_min = rospy.get_param("~hue_min", 5)
	hue_max = rospy.get_param("~hue_max", 15)
	sat_min = rospy.get_param("~sat_min", 100)
	sat_max = rospy.get_param("~sat_max", 255)
	val_min = rospy.get_param("~val_min", 100)
	val_max = rospy.get_param("~val_max", 255)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

	while not rospy.is_shutdown():
		ret, frame = cap.read()
		if not ret: break

		if flip_frame:
			frame = cv2.flip(frame, flip_code)
		blur = cv2.GaussianBlur(frame, (blur_val, blur_val), 0)
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, (hue_min, sat_min, val_min), (hue_max, sat_max, val_max))

		if cam_index == 0:
			x_target = frame_width // 2
			y_target = frame_height - 175
		else:
			x_target = frame_width // 2
			y_target = 175
		cv2.circle(hsv, (x_target, y_target), 5, (255, 255, 255), 3)
		hue, sat, val = hsv[y_target, x_target]

		hue_min_output = int(max(hue - 5, 0))
		hue_max_output = int(min(hue + 5, 255))
		sat_min_output = int(max(sat - 77, 0))
		sat_max_output = int(min(sat + 77, 255))
		val_min_output = int(max(val - 77, 0))
		val_max_output = int(min(val + 77, 255))

		cv2.namedWindow("Frame " + str(cam_index) + " HSV", cv2.WINDOW_GUI_NORMAL)
		cv2.resizeWindow("Frame " + str(cam_index) + " HSV", (300 * frame_width // frame_height), 300)
		if cam_index == 0:
			cv2.moveWindow("Frame " + str(cam_index) + " HSV", (512 - (300 * frame_width // frame_height)), 0)
		else:
			cv2.moveWindow("Frame " + str(cam_index) + " HSV", 512, 0)
		cv2.imshow("Frame " + str(cam_index) + " HSV", hsv)

		cv2.namedWindow("Frame " + str(cam_index) + " Mask", cv2.WINDOW_GUI_NORMAL)
		cv2.resizeWindow("Frame " + str(cam_index) + " Mask", (300 * frame_width // frame_height), 300)
		if cam_index == 0:
			cv2.moveWindow("Frame " + str(cam_index) + " Mask", (512 - 300 * frame_width // frame_height), 300)
		else:
			cv2.moveWindow("Frame " + str(cam_index) + " Mask", 512, 300)
		cv2.imshow("Frame " + str(cam_index) + " Mask", mask)

		rospy.Rate(50).sleep()
		if cv2.waitKey(1) & 0xFF == ord('q'): break

	rospy.loginfo("Frame " + str(cam_index) + " " + str((hue_min_output, hue_max_output, sat_min_output, sat_max_output, val_min_output, val_max_output)))
	cap.release()
	cv2.destroyAllWindows()
	rospy.spin()

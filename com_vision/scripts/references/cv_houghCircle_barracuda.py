import cv2
import numpy as np

cap = cv2.VideoCapture(0)
prevCircle = None
dist = lambda x1, y1, x2, y2: (x1-x2)**2+(y1-y2)**2

while True:
	ret, frame = cap.read()
	if not ret: break

	grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blurFrame = cv2.GaussianBlur(grayFrame, (15, 15), 0)

	circles = cv2.HoughCircles(blurFrame, cv2.HOUGH_GRADIENT, 1, 100,
	param1=100, param2=30, minRadius=5, maxRadius=200)

	if circles is not None:
		circles = np.uint16(np.around(circles))
		chosen = None
		for i in circles[0, :]:
			if chosen is None: chosen = i
			if prevCircle is not None:
				if dist(chosen[0], chosen[1], prevCircle[0], prevCircle[1]) <= dist(i[0], i[1], prevCircle[0], prevCircle[1]):
					chosen = i
		cv2.circle(blurFrame, (chosen[0], chosen[1]), 1, (0, 100, 100), 3)
		cv2.circle(blurFrame, (chosen[0], chosen[1]), chosen[2], (255, 0, 255), 3)
		prevCircle = chosen

	cv2.imshow("circles", blurFrame)

	# cv.imshow("frame", frame)
	if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()

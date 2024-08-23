import cv2
import numpy as np
print(cv2.__version__)

hue_min = 0
hue_max = 10
sat_min = 100
sat_max = 255
val_min = 100
val_max = 255

cap = cv2.VideoCapture(0)

while True:
	
	ret, frame = cap.read()
	if not ret: break
		
	blur = cv2.GaussianBlur(frame, (17, 17), 0)
	hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, (hue_min, sat_min, val_min), (hue_max, sat_max, val_max))
	
	ret2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	
	if hierarchy is not None:
		x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
		centroid = (x + w // 2, y + h // 2)
		
		kalman = cv2.KalmanFilter(4, 2, 0)
		kalman.transitionMatrix = np.array([[1, 0, 1, 0],
	    	                                [0, 1, 0, 1],
	        	                            [0, 0, 1, 0],
	            	                        [0, 0, 0, 1]], np.float32)
		kalman.measurementMatrix = np.array([[1, 0, 0, 0],
	                	                     [0, 1, 0, 0]], np.float32)
		kalman.processNoiseCov = np.array([[1, 0, 0, 0],
	                    	               [0, 1, 0, 0],
	                        	           [0, 0, 1, 0],
	                            	       [0, 0, 0, 1]], np.float32) * 1e-5
		kalman.measurementNoiseCov = np.array([[1, 0],
											[0, 1]], np.float32) * 1e-4
		kalman.predict()
		
		measurement = np.array([centroid[0], centroid[1]], np.float32)
		kalman.correct(measurement)
		
		predicted_centroid = kalman.statePost
		
		print(predicted_centroid)
		print(centroid)
		print(x, y, w, h)

		ball_radius = max(w, h) // 2
		cv2.circle(frame, centroid, ball_radius, (0, 255, 0), 5)

	cv2.imshow("Frame", frame)
	# cv2.imshow("HSV", hsv)
	cv2.imshow("Mask", mask)
	
	if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()

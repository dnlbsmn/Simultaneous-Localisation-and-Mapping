import cv2 as cv
import numpy as np

def extract_landmarks(image, ui_mode = 0):
	'''
	extract_landmarks takes an image that has closed contours and returns an array of corners

	:image: a (binary) grayscale image of the map with the outside filled
	:return: an array of the corners in the map
	'''

	landmarks = []

	harris_corners = cv.cornerHarris(image, 20, 11, 0.05)
	ret, harris_corners = cv.threshold(harris_corners, 127, 255, cv.THRESH_BINARY)
	harris_corners = harris_corners.astype(np.uint8)
	
	contours, hierarchy = cv.findContours(harris_corners, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

	for contour in contours:
		moment = cv.moments(contour)
		
		cx = int(moment["m10"] / moment["m00"])
		cy = int(moment["m01"] / moment["m00"])

		landmarks.append([cx, cy])	

	if (ui_mode):
		cv.imshow('Harris Corners', harris_corners)
		cv.waitKey(0)

		kernel = np.ones((7,7), np.uint8)
		harris_corners = cv.dilate(harris_corners, kernel, iterations= 2)

		image[harris_corners > 0.025 * harris_corners.max()] = 128
	
		cv.imshow('Harris Corners', image)
		cv.waitKey(0)
	
	return landmarks

image = cv.imread('Corner test.png')
image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

landmarks = extract_landmarks(image, ui_mode = 1)
print(len(landmarks))
print(landmarks)

cv.destroyAllWindows()

### ===================================== ###
# IMPORTING MODULES

import cv2 as cv
import numpy as np
import math
from constants import *

RAD_PER_PIXEL = 0.001554434

### ===================================== ###
# PIPELINED FUNCTIONS

# Given a view of what the bot sees returns a set of corners
def observe_landmarks(view, ui_mode = 0):
	# Extracts the landmarks from the current view
	observed_landmarks = landmarks_from_slice(view, ui_mode)
	linked_points = derectify_points(observed_landmarks)
	linked_landmarks = landmark_filter(linked_points, view, ui_mode)

	# Clears out old observed landmarks and writes new ones
	observed_landmarks = []

	for point in linked_landmarks:
		observed_landmarks.append([point[0][0] - 319, point[0][1]])
	
	if (ui_mode):
		print("New landmarks: ", observed_landmarks)

		for ol in observed_landmarks:
			view[ol[1]][ol[0]] = 0

		cv.imshow("Landmark generation", view)
		cv.waitKey(100)

	return observed_landmarks

### ===================================== ###
# IMAGE PROCESSING FUNCTIONS

# Takes an image of a global map and returns the position of landmarks
def landmarks_from_global(input_image, ui_mode = 0):

	landmarks = []

	try:
		image = cv.cvtColor(input_image, cv.COLOR_BGR2GRAY)
	except:
		image = input_image.copy()

	harris_corners = cv.cornerHarris(image, GLOBAL_CORNER_SIZE, GLOBAL_SOBEL, GLOBAL_K)
	ret, harris_corners = cv.threshold(harris_corners, 127, 255, cv.THRESH_BINARY)
	harris_corners = harris_corners.astype(np.uint8)
	
	contours, hierarchy = cv.findContours(harris_corners, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
	
	if (ui_mode):
		cv.imshow("Landmark generation", harris_corners)
		cv.waitKey(0)

		kernel = np.ones((7,7), np.uint8)
		harris_corners = cv.dilate(harris_corners, kernel, iterations= 2)

		image[harris_corners > 0.003 * harris_corners.max()] = 128
	
		cv.imshow("Landmark generation", image)
		cv.waitKey(0)

	for contour in contours:
		moment = cv.moments(contour)
		try:
			cx = int(moment["m10"] / moment["m00"])
			cy = int(moment["m01"] / moment["m00"])
		
			landmarks.append([cx, cy])	
			
			image[int(cy)][int(cx)] = 200
		except:
			print("no landmarks found")

	return landmarks

# Takes an image of a local map and returns the position of landmarks
def landmarks_from_slice(image, ui_mode = 0):

	landmarks = []

	harris_corners = cv.cornerHarris(image, LOCAL_CORNER_SIZE, LOCAL_SOBEL, LOCAL_K)

	ret, harris_corners = cv.threshold(harris_corners, THRESH_HARRIS, 255, cv.THRESH_BINARY) #90
	harris_corners = harris_corners.astype(np.uint8)
	
	contours, hierarchy = cv.findContours(harris_corners, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
			
	if (ui_mode):
		cv.imshow("Landmark generation", harris_corners)

		kernel = np.ones((3,3), np.uint8)
		harris_corners = cv.dilate(harris_corners, kernel, iterations= 2)

		image[harris_corners > 0.025 * harris_corners.max()] = 63
		
	for contour in contours:
		moment = cv.moments(contour)
		try:
			cx = int(moment["m10"] / moment["m00"])
			cy = int(moment["m01"] / moment["m00"])
		
			landmarks.append([cx, cy])	
			
			image[int(cy)][int(cx)] = 255
		except:
			print("no landmarks found")
			
	if (ui_mode):
		cv.imshow("Landmark generation", image)
	
	return landmarks

# Cleans a field of view for finding landmarks
def clean_local_map(image):
	image = cv.blur(image, (5, 5))

	ret, thresh = cv.threshold(image, 45, 255, cv.THRESH_BINARY)
	image = thresh
	image = cv.erode(image, np.ones((2, 2)), iterations = 1)

	image = cv.dilate(image, cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5)), iterations = 1)

	return image

# Cleans a global map for finding landmarks
def clean_global_map(image):
	outer_contours = [np.array([[0, 0], [0, 601], [601, 601], [601, 0]])]
	inner_contours = []

	image = cv.dilate(image, np.ones((22, 22)))

	cv.imshow("Landmark generation", image)
	cv.waitKey(0)

	contoursA, hierarchyA = cv.findContours(image, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
	contoursB, hierarchyB = cv.findContours(image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
	
	for contour in contoursB:
		if (cv.contourArea(contour) > 1000):
			outer_contours.append(contour)

	for contour in contoursA:
		if (cv.contourArea(contour) > 20):
			inner_contours.append(contour)

	print(inner_contours)

	outer_image = cv.drawContours(np.zeros((601, 601), dtype=np.uint8), outer_contours, -1,  255, cv.FILLED)
	inner_image = cv.drawContours(np.zeros((601, 601), dtype=np.uint8), inner_contours, -1,  255, cv.FILLED)

	overall_image = cv.addWeighted(inner_image, 1, outer_image, 1, 0)

	print("Overall image")
	cv.imshow("Landmark generation", overall_image)
	cv.waitKey(0)

	overall_image = cv.erode(overall_image, np.ones((5, 5)), iterations = 5)#cv.getStructuringElement(cv.MORPH_ELLIPSE, (22, 22)))

	print("Overall image")
	cv.imshow("Landmark generation", overall_image)
	cv.waitKey(0)

	return overall_image

# Cleans a map to figure out where the robot can traverse
def clean_traverse_map(image):
	inner_contours = []
	image = cv.blur(image, (5, 5))

	print("Blur")
	cv.imshow("Landmark generation", image)
	cv.waitKey(0)

	ret, inner_image = cv.threshold(image, 50, 255, cv.THRESH_BINARY)
	
	print("Threshold")
	cv.imshow("Landmark generation", inner_image)
	cv.waitKey(0)

	contours, hierarchy = cv.findContours(inner_image, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

	for contour in contours:
		if (cv.contourArea(contour) > 20):
			inner_contours.append(contour)

	inner_image = cv.drawContours(np.zeros((601, 601), dtype=np.uint8), inner_contours, -1,  255, 1)

	overall_image = cv.dilate(inner_image, cv.getStructuringElement(cv.MORPH_ELLIPSE, (2, 2)), iterations = 1)

	print("Overall image")
	cv.imshow("Landmark generation", overall_image)
	cv.waitKey(0)

	return overall_image

### ===================================== ###
# MATHEMATICAL FUNCTIONS

# This function takes a set of linked points and returns their derectified values
def derectify_points(points):
	linked_points = []
	
	for point in points:
		y = point[1]
		
		x = math.atan((point[0] - 319.5)/point[1])
		x /= RAD_PER_PIXEL
		x += 319.5
		
		derectified = [int(round(x)), int(round(y))]
		
		linked_points.append([point, derectified])
	
	return linked_points

# Filters landmarks in a single field of view
def landmark_filter(linked_landmarks, view, ui_mode):
	filtered_landmarks = []
	
	for linked in linked_landmarks:
		landmark = linked[1]
		
		if ((landmark[0] > 620) or (landmark[0] < 20) or (landmark[1] < 40) or (landmark[1] > 200)):
			print("omitted landmark: ", landmark)
			continue
		
		pass_flag = True
		for linked_check in linked_landmarks:
			land_check = linked_check[1]
		
			if (abs(landmark[0] - land_check[0]) < 40):
				if (landmark[1] > land_check[1]):
					pass_flag = False
					break
					
		if (pass_flag): 
			filtered_landmarks.append(linked)
			if (ui_mode):
				view[linked[0][1]][linked[0][0]] = 0

	return filtered_landmarks

'''
image = cv.imread("generated_map_special.png")
image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

image = clean_traverse_map(image)
cv.imshow("Landmark generation", image)
cv.waitKey(0)

image = clean_global_map(image)
cv.imshow("Landmark generation", image)
cv.waitKey(0)

lm = landmarks_from_global(image, 1)

cv.imshow("Harris Corners", image)
cv.waitKey(0)
print(lm)

image = cv.imread("super_generated_map.png")
image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

image2 = cv.imread("Imagine.png")
image2 = cv.cvtColor(image2, cv.COLOR_BGR2GRAY)

imug = cv.addWeighted(image, 1, image2, 0.5, 0)

cv.imshow("Harris Corners", imug)
cv.waitKey(0)
'''

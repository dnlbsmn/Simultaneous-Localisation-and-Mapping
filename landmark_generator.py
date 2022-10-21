import cv2 as cv
import numpy as np
from constants import *

def landmarks_from_global(image, ui_mode = 0):
	'''
	extract_landmarks takes an image that has closed contours and returns an array of corners

	:image: a (binary) grayscale image of the map with the outside filled
	:return: an array of the corners in the map
	'''

	landmarks = []

	harris_corners = cv.cornerHarris(image, GLOBAL_CORNER_SIZE, GLOBAL_SOBEL, GLOBAL_K)
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

		image[harris_corners > 0.003 * harris_corners.max()] = 128
	
		cv.imshow('Harris Corners', image)
		cv.waitKey(0)
	
	return landmarks
	
def landmarks_from_slice(image, ui_mode = 0):
	'''
	extract_landmarks takes an image that has closed contours and returns an array of corners

	:image: a (binary) grayscale image of the map with the outside filled
	:return: an array of the corners in the map
	'''

	landmarks = []

	harris_corners = cv.cornerHarris(image, LOCAL_CORNER_SIZE, LOCAL_SOBEL, LOCAL_K)
	ret, harris_corners = cv.threshold(harris_corners, 90, 255, cv.THRESH_BINARY)
	harris_corners = harris_corners.astype(np.uint8)
	
	contours, hierarchy = cv.findContours(harris_corners, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
			
	if (ui_mode):
		cv.imshow('Harris Corners', harris_corners)
		#cv.waitKey(0)

		kernel = np.ones((7,7), np.uint8)
		harris_corners = cv.dilate(harris_corners, kernel, iterations= 2)

		image[harris_corners > 0.035 * harris_corners.max()] = 128
		
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
		cv.imshow('Harris Corners', image)
	
	return landmarks
	
# Filters landmarks in a single field of view
def landmark_filter(linked_landmarks):
	filtered_landmarks = []
	
	for linked in linked_landmarks:
		landmark = linked[1]
		
		if ((landmark[0] > 625) or (landmark[0] < 15)):
			continue
		
		pass_flag = True
		for linked_check in linked_landmarks:
			land_check = linked_check[1]
		
			if (abs(landmark[0] - land_check[0]) < 30):
				if (landmark[1] > land_check[1]):
					pass_flag = False
					break
					
		if (pass_flag): 
			filtered_landmarks.append(linked)
		
	return filtered_landmarks

# Cleans a field of view for finding corners
def clean_image_for_lm(image):
	image = cv.blur(image, (5, 5))
	ret, thresh = cv.threshold(image, 40, 255, cv.THRESH_BINARY)
	image = thresh
	image = cv.erode(image, np.ones((3, 3)), iterations = 1)
	image = cv.dilate(image, np.ones((2, 2)),iterations = 1)
	
	return image

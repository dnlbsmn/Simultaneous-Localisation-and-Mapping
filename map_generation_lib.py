import numpy as np
import math
from constants import *

### ===================================== ###
# INITIALISATION

def initialise():
	corner_distribution()

### ===================================== ###
# CORNER MANIPULATION FUNCTIONS

# Takes observed landmark poses and converts them to a single cartesian plane
def map_landmarks(landmark_list, heading_offset):
	rectified_landmarks = []

	for pose in landmark_list:
		for landmark in pose[0]:
			print(landmark)

			x1 = landmark[0]
			y1 = landmark[1]

			r = math.sqrt(x1*x1 + y1*y1)
			phi = math.asin(x1/r)

			theta = (pose[1] + heading_offset) * math.pi / 180

			x = int(r*math.sin(theta + phi))
			y = int(r*math.cos(theta + phi))

			rectified_landmarks.append([x, y])

	return rectified_landmarks

### ===================================== ###
# CORNER MATCHING FUNCTIONS

# Again just a gaussian distribution function
def gaussian(x, mu, sig):
	return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

# A simple function to find the probability of a shift
def get_probability(match):
	return match[0]

# Generating the normal distribution of the corner matching
def corner_distribution():
	global corner_normal

	corner_normal = []
	distances = np.linspace(0, 500, 500)

	for distance in distances:
		corner_normal.append(gaussian(distance, 0, MAX_CORNER_ERROR / 3))

# Taking two sets of corners and returning the probability of a match excluding excess corners
def match_probability(primary_corners, secondary_corners):
	corner_probability = 1

	# Generating the product of all the corner match probabilities
	for primary in primary_corners:
		corner_distances = []
		
		for secondary in secondary_corners:
			corner_distance = int(round(math.dist(primary, secondary)))
			corner_distances.append(corner_distance)
		
		corner_probability *= corner_normal[min(corner_distances)]

	return corner_probability

# Trying to match sets of map readings
def match_corners(global_corners, local_corners):
	global corner_normal

	match_probabilities = []

	for local_corner in local_corners:
		for global_corner in global_corners:
			# , [7, 19]Finding the difference between two specific corners
			x_shift = global_corner[0] - local_corner[0]
			y_shift = global_corner[1] - local_corner[1]

			# Shifting the local map so that one corner matches exactly
			shifted_corners = []
			for corner in local_corners:
				shifted_corners.append([corner[0] + x_shift, corner[1] + y_shift])

			# Predicting the probability of a certain shift matching
			match = [match_probability(shifted_corners, global_corners), [x_shift, y_shift]]
			match_probabilities.append(match)

	# Returning the most likely match
	match_probabilities.sort(key = get_probability, reverse = True)
	return match_probabilities[0][1]

### ===================================== ###
# TEST CODE

'''
global_corners = [[0, 0], [8, 0], [8, 8]]
local_corners = [[4, 4], [12, 4], [12, 12]]

corner_distribution()

print(match_corners(global_corners, local_corners))
'''
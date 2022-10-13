import numpy as np
import cv2 as cv
import math

# initialises a particle as a dictionary of position, rotation and weight
def create_particle(x, y, rot, weight, ui_mode = 0):
	particle = {"position": [x, y], "rotation": rot, "weight": weight}

	if (ui_mode):
		display[int(particle["position"][1])][int(particle["position"][0])] = [0, 128, 128]

	return particle


# moves a particle given a linear and angular displacement
def move_particle(particle, linear, angular, ui_mode = 0):
	global display

	if (ui_mode):
		display[int(particle["position"][1])][int(particle["position"][0])] = [0, 0, 0]

	print("Linear movement: ", linear, "Angular movement: ", angular)
	particle["rotation"] += angular

	particle["position"][0] += linear * math.cos(particle["rotation"])
	particle["position"][1] += linear * math.sin(particle["rotation"])

	if (ui_mode):
		display[int(particle["position"][1])][int(particle["position"][0])] = [0, 128, 128]


# updates a particle's weight given a landmarks that it has observed
def update_particle(particle, observed_landmark, ui_mode = 0):
	particle["weight"] = 1
	
	distances = []
	weights = []

	for landmark in landmarks:
		distance = int(math.dist(landmark, observed_landmark))
		distances.append(distance)
		weight = normal[distance]
		weights.append(weight)	
	
	print(distances)
	print(weights)


# Just a gaussian distribution function
def gaussian(x, mu, sig):
	return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))


# A function to initialise the landmarks array
def initialise_landmarks(ui_mode = 0):
	global landmarks, display, normal

	normal = []
	distances = np.linspace(0, 500, 100)

	for distance in distances:
		normal.append(gaussian(distance, 0, 100))  

	landmarks = [[20, 15],[7, 60],[60, 12],[17, 36]]

	if (ui_mode):
		for lm in landmarks:
			display[lm[1]][lm[0]] = [128, 128, 0]


cv.namedWindow("display", cv.WINDOW_NORMAL)
cv.resizeWindow("display", (1280, 960))

display = np.zeros((64, 64), dtype = np.uint8)
display = cv.cvtColor(display, cv.COLOR_GRAY2BGR)		

initialise_landmarks(ui_mode = 1)

p1 = create_particle(60, 10, 1, 1, ui_mode = 1)

display[60][9] = [128, 0, 128]
print(landmarks)
update_particle(p1, [9, 60], ui_mode = 1)

cv.imshow("display", display)
cv.waitKey(0)

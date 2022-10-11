import cv2 as cv
import numpy as np
import math

### ===================================== ###
# TUNING CONSTANTS

MOTOR_DRIFT_MAX = 0.015
ANGLE_ERROR_MAX = 0.1
LINEAR_ERROR_MAX = 0.1

### ===================================== ###
# FUNCTIONS

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


# Generates a particle given an attempt to move the bot
def generate_particle(parent, linear, angular):
    child = {"position": [0, 0], "rotation": 0, "weight": 1}

    a = parent["rotation"]
    x = parent["position"][0] 
    y = parent["position"][1]

    # The uncertainty from bot motor imbalance
    a += np.random.normal(0, MOTOR_DRIFT_MAX, None)

    # The uncertainty from explicit movements
    a += angular + angular * np.random.normal(0, ANGLE_ERROR_MAX / 3, None)
    x += linear * math.cos(a) * ( 1 +  np.random.normal(0, LINEAR_ERROR_MAX / 3, None))
    y += linear * math.sin(a) * ( 1 + np.random.normal(0, LINEAR_ERROR_MAX / 3, None))
    
    # Assigning the generated values appropriately
    child["rotation"] = a
    child["position"][0] = x
    child["position"][1] = y
    
    return child


# Generates an example distribution of a movement
def example_distributions():
    p1 = {"position": [31, 0], "rotation": math.pi / 2, "weight": 1}

    for i in range(300):
        p = generate_particle(p1, 30, 0.5)
        display_particle(p, 5)

    cv.imshow("display", display)
    cv.waitKey(0)


# Initialises particles to begin particle filtering
def initialise_particles(particle_count):
	global display, particles

	cv.namedWindow("display", cv.WINDOW_NORMAL)
	cv.resizeWindow("display", (640, 480)) 
	display = np.zeros((64, 64), dtype = np.uint8)

	particles = []
	parent = {"position": [31, 31], "rotation": 0, "weight": 0}

	for i in range(particle_count):
		p = generate_particle(parent, 20, 20 * math.pi)
		display_particle(p, 255)
		p["weight"] = np.random.rand()
		particles.append(p)

	cv.imshow("display", display)
	cv.waitKey(0)


# Displays the particles on the matrix
def display_particle(p, increment):
    global display

    if (display[int(round(p["position"][1]))][int(round(p["position"][0]))] != 255): 
        display[int(round(p["position"][1]))][int(round(p["position"][0]))] += increment


# Takes the current set of particles and resamples them
def resample(particle_count, threshold):
	global display, particles

	total_weight = 0
	resample = []

	for particle in particles:
		if (particle["weight"] > threshold):
			resample.append(particle)
			total_weight += particle["weight"]

	for particle in resample:
		particle["weight"] /= total_weight

	particles = resample

	for particle in particles:
		populate_particle(particle, 5)
		
# Populate more particles around a given particle
def populate_particle(parent, count):
	global things

	for i in range(count):
		x = parent["position"][0] + np.random.normal(0, 0.1, None)
		y = parent["position"][1] + np.random.normal(0, 0.1, None)
		rot = parent["rotation"] + np.random.normal(0, 0.01, None)

		p = create_particle(x, y, rot, 1)
		things.append(p)

### ===================================== ###
# TEST CODE

things = []
initialise_particles(10)

resample(10, 0.7)

print("###")
display = np.zeros((64, 64), dtype = np.uint8)

for particle in particles:
	display_particle(particle, 255)
	print(particle)

cv.imshow("display", display)
cv.waitKey(0)

for thing in things:
	print(thing)

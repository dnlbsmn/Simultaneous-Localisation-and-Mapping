import cv2 as cv
import numpy as np
import math
from constants import *

### ===================================== ###
# INITIALISATION FUNCTIONS
	
# A function to initialise the landmarks array and the gaussian distance
def initialise(ui_mode):
	global landmarks, display, normal

	normal = []
	distances = np.linspace(0, 500, 500)

	# Generate a uniformly distributed set of particles inside the maze
	initialise_particles(INIT_PARTICLES, ui_mode)
	
	for distance in distances:
		normal.append(gaussian(distance, 0, MAX_LANDMARK_ERROR / 3))  
	
	if (ui_mode):
		for lm in landmarks:
			display[lm[1]][lm[0]] = [128, 128, 0]

# initialises a particle as a dictionary of position, rotation and weight
def create_particle(x, y, rot, weight, ui_mode = 0):
	particle = {"position": [x, y], "rotation": rot, "weight": weight}

	if (ui_mode):
		display[int(particle["position"][1])][int(particle["position"][0])] = [255, 255, 255]

	return particle

# Just a gaussian distribution function
def gaussian(x, mu, sig):
	return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

# Initialising the display matrix
def initialise_display(image):
	global display

	cv.namedWindow("display", cv.WINDOW_NORMAL)
	cv.resizeWindow("display", (1280, 960))
	display = image
	display = cv.cvtColor(display, cv.COLOR_GRAY2BGR)

### ===================================== ###
# INITIALISE STEP

# Populate the map with a given number of particles
def initialise_particles(count, ui_mode = 0):
	global particles

	particles = []

	for i in range(count):
		x = np.random.uniform(0, 230, None)
		y = np.random.uniform(0, 286, None)
		rot = np.random.uniform(0, 2 * math.pi, None)
		
		p = create_particle(x, y, rot, 1)

		if (ui_mode):
			try:
				display[int(p["position"][1])][int(p["position"][0])] = [255, 255, 255]
			except:
				print("initialised particle out of range")			
			
		particles.append(p)

### ===================================== ###
# MOVE STEP

# Iterates through all the particles moving them according to input
def move_step(linear, angular, ui_mode = 0):
	global particles
	
	resample = []
	
	for particle in particles:
		resample.append(move_particle(particle, linear, angular, ui_mode))

	particles = resample

# Generates a particle given an attempt to move the bot
def move_particle(parent, linear, angular, ui_mode = 0):
	child = {"position": [0, 0], "rotation": 0, "weight": 1}

	if (ui_mode):
		try:
			display[int(parent["position"][1])][int(parent["position"][0])] = [0, 0, 0]
		except:
			print("particle moved out of display area")

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
	
	if (ui_mode):
		try:
			display[int(child["position"][1])][int(child["position"][0])] = [0, 255, 255]
		except:
			print("particle moved out of display area")

	return child

### ===================================== ###
# UPDATE STEP

# Iterates through all the particles updating their weights according to landmarks
def update_step(observed_landmarks, ui_mode = 0):
	for particle in particles:
		update_particle(particle, observed_landmarks, ui_mode)

# updates a particle's weight given a landmarks that it has observed
def update_particle(particle, observed_landmarks, ui_mode = 0):
	global landmarks, display

	for observed_landmark in observed_landmarks:
		rx = observed_landmark[0]
		ry = observed_landmark[1]

		r = math.sqrt(rx * rx + ry * ry)
		if (rx != 0):
			phi = math.atan(ry / rx)
		else:
			phi = math.pi / 2

		x = particle["position"][0] + math.copysign(r, rx) * math.cos(particle["rotation"] + phi)
		y = particle["position"][1] + math.copysign(r, ry) * math.sin(particle["rotation"] + phi)

		distances = []

		for landmark in landmarks:
			distance = int(math.dist(landmark, [x, y]))
			distances.append(distance)
		
		particle["weight"] *= normal[min(distances)]

	if (ui_mode):
		print(particle)

### ===================================== ###
# RESAMPLE STEP

# outputs the weight of a given particle
def particle_weight(particle):
	return particle["weight"]

# Takes the current set of particles and resamples them
def trim_particles(particle_count, ui_mode = 0):
	global display, particles

	total_weight = 0
	resample = []

	particles.sort(key = particle_weight, reverse = True)

	if (ui_mode):
		for particle in particles[-particle_count:]:
			display[int(particle["position"][1])][int(particle["position"][0])] = [0, 0, 0]

	particles = particles[:particle_count]

	for particle in particles:
		total_weight += particle["weight"]
		if (ui_mode):
			display[int(particle["position"][1])][int(particle["position"][0])] = [0, 255, 0]

	for particle in particles:
		particle["weight"] /= total_weight

	if (ui_mode):
		for particle in particles:
			print(particle)

# Resampling a trimmed set of particles
def resample_particles(total_count, ui_mode = 0):
	global particles

	count = total_count - len(particles)

	resample = []

	for particle in particles:
		particle_count = int(round(particle["weight"] * count))
		propagate_particle(particle, particle_count, resample, ui_mode)

	particles = resample
	
# Randomly propagate more particles around a parent particle 
def propagate_particle(parent, count, resample, ui_mode = 0):
	global display, particles

	for i in range(count - 1):
		x = parent["position"][0] + np.random.normal(0, LINEAR_PROP_MAX / 3, None)
		y = parent["position"][1] + np.random.normal(0, LINEAR_PROP_MAX / 3, None)
		rot = parent["rotation"] + np.random.normal(0, ANGULAR_PROP_MAX / 3, None)
		
		particle = create_particle(x, y, rot, 1)

		if (ui_mode):
			try:
				display[int(particle["position"][1])][int(particle["position"][0])] = [0, 0, 255]		
			except:
				print("particle propagated outside of display bounds")

		resample.append(particle)
	
### ===================================== ###
# MISCELLANEOUS

# Displays the particles on the matrix
def display_particle(p, color):
	global display

	display[int(round(p["position"][1]))][int(round(p["position"][0]))] = color

# Generates an example distribution of a movement
def example_distributions():
	p1 = {"position": [31, 0], "rotation": math.pi / 2, "weight": 1}

	for i in range(300):
		p = move_particle(p1, 30, 0.5)
		display_particle(p, 5)

	cv.imshow("display", display)
	cv.waitKey(0)

### ===================================== ###
# PARTICLE FILTER

# The overall pipeline for particle filtering
def particle_filter(observed_landmarks, ui_mode = 0):
	global landmarks, particles, display

	# Read the observed landmarks of the output
	
	# Move each particle a given distance

	# Update the particle weights accordingly
	update_step(observed_landmarks, ui_mode = 0)

	# Trim the unlikely particles
	trim_particles(TRIMMED_PARTICLES, ui_mode = 0)

	# Propagate the remaining particles according to weight
	resample_particles(RESAMPLED_PARTICLES, ui_mode = 1)

	if (ui_mode):
		cv.imshow("display", display)
		cv.waitKey(0)
		

	# Wait for the next movement and repeat

### ===================================== ###
# TEST CODE
'''
landmarks = [[63, 63], [10, 10], [10, 117]]

initialise_display()
initialise(ui_mode = 1)

particle_filter([[-30, -30], [23, 23], [-30, 77]], ui_mode = 1)

print("DONE")
'''

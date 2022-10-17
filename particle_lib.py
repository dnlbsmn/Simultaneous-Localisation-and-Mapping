import cv2 as cv
import numpy as np
import math
from constants import *

### ===================================== ###
# INITIALISATION FUNCTIONS
	
# A function to initialise the landmarks array and the gaussian distance
def initialise(map_image, ui_mode = 0):
	global landmarks, display, normal, obstacle_map

	obstacle_map = cv.cvtColor(map_image, cv.COLOR_BGR2GRAY)

	normal = []
	distances = np.linspace(0, 500, 500)

	# Generate a uniformly distributed set of particles inside the maze
	initialise_particles(INIT_PARTICLES, ui_mode)
	
	for distance in distances:
		normal.append(gaussian(distance, 0, MAX_LANDMARK_ERROR / 3))  
	
	if (ui_mode):
		for lm in landmarks:
			display[lm[1]][lm[0]] = [128, 128, 0]
		
		cv.imshow("display", display)
		cv.waitKey(0)

# initialises a particle as a dictionary of position, rotation and weight. Returns none if position is invalid
def create_particle(x, y, rot, weight, ui_mode = 0):
	global obstacle_map

	try:
		if (obstacle_map[int(y)][int(x)] == 255):
			return None
	except:
		return None

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

		if (p == None):
			continue

		if (ui_mode):
			try:
				display[int(p["position"][1])][int(p["position"][0])] = [64, 0, 64]
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
		moved_particle = move_particle(particle, linear, angular, ui_mode)

		if (moved_particle != None):
			resample.append(moved_particle)

	particles = resample
	
	if (ui_mode):
		cv.imshow("display", display)
		cv.waitKey(0)

# Generates a particle given an attempt to move the bot
def move_particle(parent, linear, angular, ui_mode = 0):
	angular = (math.pi / 180) * angular 

	if (ui_mode):
		try:
			display[int(parent["position"][1])][int(parent["position"][0])] = [40, 40, 40]
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
	child = create_particle(x, y, a, 1, ui_mode = ui_mode)

	if (child == None):
		return None

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

### ===================================== ###
# RESAMPLE STEP

# The entirety of the resample step of particle filtering
def resample_step(total_count = RESAMPLED_PARTICLES, remnant_count = TRIMMED_PARTICLES, remnant_weight = TRIM_THRESHOLD, ui_mode = 0):
	trim_particles(remnant_count, remnant_weight, ui_mode)
	resample_particles(total_count, ui_mode)

	if (ui_mode):
		cv.imshow("display", display)
		cv.waitKey(0)

# outputs the weight of a given particle
def particle_weight(particle):
	return particle["weight"]

# Takes the current set of particles and resamples them
def trim_particles(particle_count, threshold, ui_mode = 0):
	global display, particles

	total_weight = 0
	resampled = []
	unsampled = []

	particles.sort(key = particle_weight, reverse = True)

	i = 0

	for particle in particles:
		if ((particle["weight"] > 0.8) or (i < particle_count)):
			resampled.append(particle)
			i += 1
		else:
			break

	unsampled = particles[i:]

	for particle in resampled:
		total_weight += particle["weight"]

	for particle in resampled:
		particle["weight"] /= total_weight

	if (ui_mode):
		for particle in unsampled:
			display[int(particle["position"][1])][int(particle["position"][0])] = [0, 0, 0]
		for particle in resampled:
			display[int(particle["position"][1])][int(particle["position"][0])] = [0, 255, 0]
		print("Remainder particles: ", len(resampled))

	particles = resampled

# Resampling a trimmed set of particles
def resample_particles(total_count, ui_mode = 0):
	global particles

	count = total_count - len(particles)

	resample = []

	for particle in particles:
		particle_count = int(round(particle["weight"] * count))
		propagate_particle(particle, particle_count, resample, ui_mode)

	particles = resample

	if (ui_mode):
		print("Resample particles: ", len(particles))
	
# Randomly propagate more particles around a parent particle 
def propagate_particle(parent, count, resample, ui_mode = 0):
	global display, particles

	for i in range(count - 1):
		x = parent["position"][0] + np.random.normal(0, LINEAR_PROP_MAX / 3, None)
		y = parent["position"][1] + np.random.normal(0, LINEAR_PROP_MAX / 3, None)
		rot = parent["rotation"] + np.random.normal(0, ANGULAR_PROP_MAX / 3, None)
		
		particle = create_particle(x, y, rot, 1)

		if (particle == None):
			continue

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
	p1 = {"position": [100, 100], "rotation": math.pi / 2, "weight": 1}

	for i in range(300):
		p = move_particle(p1, 30, 0.5)
		display_particle(p, [255, 255, 0])

	cv.imshow("display", display)
	cv.waitKey(0)

### ===================================== ###
# PARTICLE FILTER

# The overall pipeline for particle filtering
def particle_filter(observed_landmarks, ui_mode = 0):
	global display

	# Read the observed landmarks of the output
	# Move each particle a given distance
	# Update the particle weights accordingly
	# Trim the unlikely particles
	# Propagate the remaining particles according to weight

	if (ui_mode):
		cv.imshow("display", display)
		cv.waitKey(0)
		
	# Wait for the next movement and repeat

### ===================================== ###
# NOTES

# Initialised particles = PURPLE
# Moved particles = BLUE
# Propagated particles = RED
# Global landmarks = CYAN
# Ghost landmarks = DARK GRAY
# Obstacles = WHITE
# Open space = BLACK

### ===================================== ###
# TEST CODE

'''
landmarks = [[63, 63], [10, 10], [10, 117]]

image = cv.imread("perfect_map.png")

observed_landmarks = [[0, 0], [0, 50]]

initialise_display(image)
initialise(image, ui_mode = 1)

for i in range(10):
	move_step(0, 0, ui_mode = 1)
	update_step(observed_landmarks, ui_mode = 1)
	resample_step(ui_mode = 1)

print("DONE")
'''



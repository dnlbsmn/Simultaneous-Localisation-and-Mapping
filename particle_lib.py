import cv2 as cv
import numpy as np
import math

### ===================================== ###
# TUNING CONSTANTS

# When moving what percentage error is allowed
MOTOR_DRIFT_MAX = 0.015
ANGLE_ERROR_MAX = 0.1
LINEAR_ERROR_MAX = 0.1

# When propogating what is the maximum error allowed
LINEAR_PROP_MAX = 20
ANGULAR_PROP_MAX = 0.1

### ===================================== ###
# INITIALISATION FUNCTIONS

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
def initialise_display():
	global display

	cv.namedWindow("display", cv.WINDOW_NORMAL)
	cv.resizeWindow("display", (1280, 960))
	display = np.zeros((128, 128), dtype = np.uint8)
	display = cv.cvtColor(display, cv.COLOR_GRAY2BGR)

	
# A function to initialise the landmarks array
def initialise_landmarks(ui_mode = 0):
	global landmarks, display, normal

	normal = []
	distances = np.linspace(0, 500, 500)
	
	for distance in distances:
		normal.append(gaussian(distance, 0, 100))  
	
	landmarks = [[4, 3], [0, 0]]
	
	if (ui_mode):
		for lm in landmarks:
			display[lm[1]][lm[0]] = [128, 128, 0]


# Initialises particles to begin particle filtering
def initialise_particles(particle_count):
	global display, particles

	particles = []
	parent = {"position": [63, 63], "rotation": 0, "weight": 0}

	populate_particles(parent, particle_count)	


# Populate more particles around a given particle
def populate_particles(parent, count):
	global particles

	for i in range(count):
		x = parent["position"][0] + np.random.normal(0, 16, None)
		y = parent["position"][1] + np.random.normal(0, 16, None)
		rot = parent["rotation"] + np.random.normal(0, 6, None)
		
		p = create_particle(x, y, rot, 1)
		display[int(p["position"][1])][int(p["position"][0])] = [255, 255, 255]

		particles.append(p)

### ===================================== ###
# MOVE STEP

# Generates a particle given an attempt to move the bot
def move_particle(parent, linear, angular, ui_mode = 0):
	child = {"position": [0, 0], "rotation": 0, "weight": 1}

	if (ui_mode):
		display[int(parent["position"][1])][int(parent["position"][0])] = [255, 0, 0]

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
		display[int(child["position"][1])][int(child["position"][0])] = [0, 255, 255]

	return child

### ===================================== ###
# UPDATE STEP

# updates a particle's weight given a landmarks that it has observed
def update_particle(particle, observed_landmarks, ui_mode = 0):
	global landmarks, display

	for observed_landmark in observed_landmarks:
		rx = observed_landmark[0]
		ry = observed_landmark[1]

		r = math.sqrt(rx * rx + ry * ry)
		phi = math.atan(ry / rx)

		x = particle["position"][0] + math.copysign(r, rx) * math.cos(particle["rotation"] + phi)
		y = particle["position"][1] + r * math.sin(particle["rotation"] + phi)
		
		display[int(y)][int(x)] = [128, 0, 128]

		distances = []

		for landmark in landmarks:
			distance = int(math.dist(landmark, [x, y]))
			print("dist: ", distance)
			distances.append(distance)
		
		print("min: ", min(distances))
		print(len(normal))
		particle["weight"] *= normal[min(distances)]

### ===================================== ###
# RESAMPLE STEP

# Takes the current set of particles and resamples them
def trim_particles(threshold):
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

# Resampling a trimmed set of particles
def resample_particle(total_count):
	global particles

	count = total_count - len(particles)

	for particle in particles:
		particle_count = math.round(particle["weight"] * count)
		propagate_particle(particle, particle_count)
	
# Randomly propagate more particles around a parent particle 
def propagate_particle(particle, count, ui_mode = 1):
	global display, particles
	
	for i in range(count - 1):
		parent["position"][0] += np.random.normal(0, LINEAR_PROP_ERROR, None)
		parent["position"][1] += np.random.normal(0, LINEAR_PROP_ERROR, None)
		parent["rotation"] += np.random.normal(0, ANGULAR_PROP_ERROR, None)
		
		p = create_particle(x, y, rot, 1)
		display[int(p["position"][1])][int(p["position"][0])] = [255, 255, 255]

		particles.append(p)

		if (ui_mode):
			display[int(p["position"][1])][int(p["position"][0])] = [0, 128, 128]
	
### ===================================== ###
# MISCELLANEOUS

# Displays the particles on the matrix
def display_particle(p, increment):
	global display

	display[int(round(p["position"][1]))][int(round(p["position"][0]))] = increment


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

def particle_filter():
	# Generate a uniformly distributed set of particles inside the maze
	
	# Move each particle a given distance
	# Read the observed landmarks of the output
	# Update the particle weights accordingly
	# Trim the unlikely particles
	# Propagate the remaining particles according to weight
	# Wait for the next movement and repeat
	print("particle filter")

### ===================================== ###
# TEST CODE

things = []

initialise_display()
initialise_landmarks(ui_mode = 1)
#initialise_particles(1)

cv.imshow("display", display)
cv.waitKey(0)

p = create_particle(63, 63, 0, 1)
display_particle(p, [255, 255, 255])

update_particle(p, [[-5, 1]], ui_mode = 1)

cv.imshow("display", display)
cv.waitKey(0)



"""
for movement in [[30, math.pi], [20, -math.pi / 2], [40, 0]]:
	
	for particle in particles:
		move_particle(particle, movement[0], movement[1], ui_mode = 1)

	cv.imshow("display", display)
	cv.waitKey(0)

	for particle in particles:
		update_particle(particle, [[5, 12]])

	#resample(10, 0.5)

	for particle in particles:
		trim_particles(0.1)
		resample_particle(10)

	cv.imshow("display", display)
	cv.waitKey(0)
"""
print("DONE")

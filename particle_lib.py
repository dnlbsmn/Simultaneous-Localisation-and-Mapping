import cv2 as cv
import numpy as np
import math

### ===================================== ###
# TUNING CONSTANTS

MOTOR_DRIFT_MAX = 0.015
ANGLE_ERROR_MAX = 0.1
LINEAR_ERROR_MAX = 0.1

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


# A function to initialise the landmarks array
def initialise_landmarks(ui_mode = 0):
    global landmarks, display, normal

    cv.namedWindow("display", cv.WINDOW_NORMAL)
    cv.resizeWindow("display", (1280, 960))
    display = np.zeros((128, 128), dtype = np.uint8)
    display = cv.cvtColor(display, cv.COLOR_GRAY2BGR)

    normal = []
    distances = np.linspace(0, 500, 100)
    
    for distance in distances:
        normal.append(gaussian(distance, 0, 100))  
    
    landmarks = [[52, 57],[39, 70],[73, 42],[71, 36]]
    
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
def generate_particle(parent, linear, angular, ui_mode = 0):
    child = {"position": [0, 0], "rotation": 0, "weight": 1}

    if (ui_mode):
        display[int(parent["position"][1])][int(parent["position"][0])] = [0, 0, 0]

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

### ===================================== ###
# UPDATE STEP

# updates a particle's weight given a landmarks that it has observed
def update_particle(particle, observed_landmark, ui_mode = 0):
	global landmarks

	for landmark in landmarks:
		distance = int(math.dist(landmark, observed_landmark))
		particle["weight"] *= normal[distance]

### ===================================== ###
# RESAMPLE STEP

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
        p = generate_particle(p1, 30, 0.5)
        display_particle(p, 5)

    cv.imshow("display", display)
    cv.waitKey(0)


### ===================================== ###
# TEST CODE

things = []

initialise_landmarks(ui_mode = 1)
initialise_particles(5)

cv.imshow("display", display)
cv.waitKey(0)

while True:
    for particle in particles:
        generate_particle(particle, 9, 0.5, ui_mode = 1)

    cv.imshow("display", display)
    cv.waitKey(0)

    for particle in particles:
        update_particle(particle, particle["position"])

    #resample(10, 0.5)

    cv.imshow("display", display)
    cv.waitKey(0)

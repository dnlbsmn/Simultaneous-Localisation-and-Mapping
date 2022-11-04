import cv2 as cv
import math
import numpy as np

import slamBotHD as sb
from time import sleep

RAD_PER_PIXEL = 0.001554434
RAD_OFFSET = 0.497418837
   
### ===================================== ###
# INITIALISATION FUNCTIONS

# Initialize or reset the matrices
def initialise_matrices():
	global slit_array, tan_array, local_map, global_map, global_preview

	slit_array = np.zeros((640), dtype = np.uint16)
	tan_array = np.zeros((640), dtype = np.float)
	local_map = np.zeros((601, 601), dtype = np.uint8)
	global_map = np.zeros((601, 601), dtype = np.uint8)
	global_preview = np.zeros((601, 601), dtype = np.uint8)

# Initialises the tangent array to be applied to the slit
def initialise_tan():
	for pixel in range(0, 320):
		tan_array[pixel] = - math.tan((RAD_PER_PIXEL * pixel - RAD_OFFSET))

### ===================================== ###
# SLIT MANIPULATION

# Fully manipulate a rectified slit image
def interpret_slit(gmap, slit_array, x, y, heading, weight):
	heading = -heading * math.pi / 180

	# Manipulating the left side
	for pixel in range(320):
		if (slit_array[319 - pixel] > 20):
			pix_y = slit_array[319 - pixel]
			pix_x = slit_array[319 - pixel] * tan_array[319 - pixel]

			r = math.sqrt(pix_x * pix_x + pix_y * pix_y)
			phi = math.asin(pix_x/r)

			rot_pix_y = int(r*math.cos(- heading + phi))
			rot_pix_x = int(r*math.sin(- heading + phi))

			gmap[rot_pix_y + y][x + rot_pix_x] = weight
			
	# Manipulating the right side
	for pixel in range(320):
		if (slit_array[639 - pixel] > 20):
			pix_y = slit_array[639 - pixel]
			pix_x = slit_array[639 - pixel] * tan_array[pixel]

			r = math.sqrt(pix_x * pix_x + pix_y * pix_y)
			phi = math.asin(pix_x/r)
			
			rot_pix_y = int(r*math.cos(heading + phi))
			rot_pix_x = int(r*math.sin(heading + phi))

			gmap[rot_pix_y + y][- rot_pix_x + x + 1] = weight 
	
# Display an uncorrected slit as a top down view
def uncorrected_slit(slit_array):
	uncorrected_view = np.zeros((255, 640), dtype = np.uint8)
	
	for pixel in range(640):
		uncorrected_view[int(slit_array[pixel])][pixel] = 255
		
	return uncorrected_view
	
# Takes the average of an eight pixel thick slice
def average_slice(slyce):
	averaged_slice = np.zeros(640, dtype=np.float)
	
	#for j in range(640):
	#	for i in range(8):
	#		averaged_slice[j] += slyce[i][j]/8

	for x in range(640):
		sum = 0
		count = 0

		for y in range(8):
			if (slyce[y][x] > 0):
				sum += slyce[y][x]
				count += 1

		if (count != 0):
			averaged_slice[x] = sum / count
		else:
			averaged_slice[x] = "Nan"

	return averaged_slice
	
# Converts the depth readings from metres to centimetres
def depth_to_centimetres(depth_array):
	centimetres_array = depth_array * 100
	centimetres_array = centimetres_array.astype(np.uint8)

	return centimetres_array

# Takes an eight thick slice from the depth sensor and formats it
def convert_to_slice(depth_image):
	slyce = depth_image[236:244][:]
	slyce = depth_to_centimetres(slyce)
	slyce_1d = average_slice(slyce)
	for i in range(len(slyce_1d)):
		if (slyce_1d[i] > 205):
			slyce_1d[i] = None

	return slyce_1d
	
# This function displays the rectified version of the slit, but it is not rotatable
def project_slit(slit_array):

	for pixel in range(320):
		if (slit_array[319 - pixel] > 0):
			y = slit_array[319 - pixel]
			x = 320 + slit_array[319 - pixel] * tan_array[319 - pixel]

			local_map[int(y)][int(x)] = 255

	for pixel in range(320):
		if (slit_array[639 - pixel] > 0):
			y = slit_array[639 - pixel]
			x = 319 - slit_array[639 - pixel] * tan_array[pixel]

			local_map[int(y)][int(x)] = 255

'''
initialise_matrices()
initialise_tan()

sb.startUp()
sb.readCoreData()
sleep(2)

depth_array = sb.imgDepth

cv.imshow("window", depth_array)
cv.waitKey(0)

depth_slice = convert_to_slice(depth_array)
interpret_slit(global_map, depth_slice, 300, 300, 150, 255)

cv.imshow("window", global_map)
cv.waitKey(0)
'''
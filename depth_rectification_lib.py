import cv2 as cv
import math
import numpy as np

RAD_PER_PIXEL = 0.001554434
RAD_OFFSET = 0.497418837
   
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
	
	for j in range(640):
		for i in range(8):
			averaged_slice[j] += slyce[i][j]/8

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

### =============================================== ###
# UNUSED FUNCTIONS

# Simulate obstacles
def test_slit():
    for pixel in range(0, 640):
        slit_array[pixel] = 500
    for pixel in range(100, 240):
        slit_array[pixel] = 500 + 100 - pixel
    for pixel in range(420, 500):
        slit_array[pixel] = 420-420 + pixel
        
# Fully manipulate a rectified slit image but in the old mirrored way
def interpret_slit_mirror(slit_array, x, y, heading):
	# Manipulating the left side
    for pixel in range(320):
        if (slit_array[319 - pixel] > 0):
            pix_y = float(slit_array[319 - pixel])
            pix_x = slit_array[319 - pixel] * tan_array[319 - pixel]

            r = math.sqrt(pix_x * pix_x + pix_y * pix_y)
            phi = math.asin(pix_x/r)

            rot_pix_y = int(r*math.cos(- heading + phi))
            rot_pix_x = int(r*math.sin(- heading + phi))

            global_map[rot_pix_y + y][x + rot_pix_x] = 255
            
    # Manipulating the right side
    for pixel in range(320):
        if (slit_array[639 - pixel] > 0):
            pix_y = float(slit_array[639 - pixel])
            pix_x = slit_array[639 - pixel] * tan_array[pixel]

            r = math.sqrt(pix_x * pix_x + pix_y * pix_y)
            phi = math.asin(pix_x/r)
            
            rot_pix_y = int(r*math.cos(heading + phi))
            rot_pix_x = int(r*math.sin(heading + phi))

            global_map[rot_pix_y + y][- rot_pix_x + x + 1] = 255 

# Clean up noise in generated map
def clean_image(map):
    map = cv.blur(map, (8, 8))
    cv.imshow("display", map)
    cv.waitKey(0)

    ret, thresh = cv.threshold(map, 30, 255, cv.THRESH_BINARY)
    map = thresh
    cv.imshow("display", map)
    cv.waitKey(0)

    map = cv.erode(map, np.ones((4, 4)), iterations = 1)
    cv.imshow("display", map)
    cv.waitKey(0)

    map = cv.dilate(map, np.ones((4, 4)),iterations = 1)
    cv.imshow("display", map)
    cv.waitKey(0)
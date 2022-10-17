import cv2 as cv
import math
import numpy as np
#from scipy.ndimage.interpolation import rotate

RAD_PER_PIXEL = 0.001554434
RAD_OFFSET = 0.497418837
   
# Initialize or reset the matrices
def initialise_matrices(resolution):
    global slit_array, tan_array, local_map, global_map

    slit_array = np.zeros((640), dtype = np.uint16)
    tan_array = np.zeros((640), dtype = np.float)
    #local_map = np.zeros((int(2.2 * resolution) + 1, int(2.2 * resolution) + 1), dtype = np.uint8)
    local_map = np.zeros((601, 601), dtype = np.uint8)
    global_map = np.zeros((601, 601), dtype = np.uint8)
    

# Simulate obstacles
def test_slit():
    for pixel in range(0, 640):
        slit_array[pixel] = 500
    for pixel in range(100, 240):
        slit_array[pixel] = 500 + 100 - pixel
    for pixel in range(420, 500):
        slit_array[pixel] = 420-420 + pixel
    #slit_array[400] = None

# Initialises the tangent array to be applied to the slit
def initialise_tan():
    for pixel in range(0, 320):
        tan_array[pixel] = - math.tan((RAD_PER_PIXEL * pixel - RAD_OFFSET))

# Fully manipulate a rectified slit image
def interpret_slit(slit_array, x, y, heading):
    #x = x - 0.5 REMEMBER

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
    #cv.imshow("display", local_map)
    
# Display an uncorrected slit as a top down view
def uncorrected_slit(slit_array):
	uncorrected_view = np.zeros((255, 640), dtype = np.uint8)
	
	for pixel in range(640):
		uncorrected_view[int(slit_array[pixel])][pixel] = 255
		
	return uncorrected_view
	
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

		
def avg_slice(slyce):
	slyce_1d = np.zeros(640, dtype=np.float)
	#print(slyce_1d)
	#print(slyce)
	
	for j in range(640):
		for i in range(8):
			slyce_1d[j] += slyce[i][j]/8
	return slyce_1d
	
# Since the depth sensor has a maximum range of 5 metres
def depth_to_greyscale(depth_array):
	greyscale_array = depth_array * 50
	greyscale_array = greyscale_array.astype(np.uint8)

	return greyscale_array
	
# This function displays the rectified version of the slit, but it is not rotatable
def project_slit(slit_array):

    for pixel in range(320):
        y = float(slit_array[319 - pixel])
        x = slit_array[319 - pixel] * tan_array[319 - pixel]

        local_map[int(y)][int(319 - x)] = 255

    for pixel in range(320):
        y = float(slit_array[639 - pixel])
        x = slit_array[639 - pixel] * tan_array[pixel]

        local_map[int(y)][int(x + 320)] = 255
        
def derectify_points(points):
	linked_points = []
	
	for point in points:
		y = point[1]
		
		x = math.atan((point[0] - 319.5)/point[1])
		x /= RAD_PER_PIXEL
		x += 319.5
		
		derectified = [int(round(x)), int(round(y))]
		
		linked_points.append([point, derectified])
	
	return linked_points


"""
cv.namedWindow("display", cv.WINDOW_NORMAL)
cv.resizeWindow("display", 640, 480)

initialise_matrices(600)
initialise_tan()

test_slit(slit_array)

interpret_slit(slit_array, 601, 601, -0.4)
uncorrected_slit(slit_array)

cv.imshow("display", uncorrected_view)
cv.waitKey(0)

interpret_slit(slit_array, 601, 601, -0.4)
interpret_slit(slit_array, 601, 601, -0.41)
interpret_slit(slit_array, 601, 601, -0.415)
interpret_slit(slit_array, 601, 601, -0.42)
interpret_slit(slit_array, 601, 601, -0.425)
interpret_slit(slit_array, 602, 602, -0.425)
interpret_slit(slit_array, 604, 601, -0.425)
interpret_slit(slit_array, 601, 601, -0.425)
interpret_slit(slit_array, 601, 601, -0.43)
interpret_slit(slit_array, 601, 601, -0.457)
interpret_slit(slit_array, 605, 601, -0.5)
interpret_slit(slit_array, 599, 608, -0.49)

cv.imshow("display", local_map)
cv.waitKey(0)

clean_image(local_map)

cv.destroyAllWindows()
"""
### =============================================== ###
# UNUSED FUNCTIONS



import cv2 as cv
import numpy as np

def process_debug(image_path):
    black = np.zeros((3100, 3100), dtype=np.uint8)
    maze = cv.imread(image_path)

    cv.namedWindow('map', cv.WINDOW_NORMAL)
    cv.resizeWindow('map', 600, 600)

    print("STEP 1")

    cv.imshow('map', maze)
    cv.waitKey(0)

    print("STEP 2")
    
    thresh, maze = cv.threshold(maze, 140, 255, cv.THRESH_BINARY_INV)
    maze = cv.cvtColor(maze, cv.COLOR_BGR2GRAY)

    cv.imshow('map', maze)
    cv.waitKey(0)

    print("STEP 3")

    kernel = np.ones((71, 71), dtype = 'uint8')

    maze = cv.dilate(maze, kernel, iterations = 1)

    cv.imshow('map', maze)
    cv.waitKey(0)

    print("STEP 4")

    maze = cv.resize(maze, (300, 300), interpolation = 2)

    cv.imshow('map', maze)
    cv.waitKey(0)

    print("STEP 5")

    thresh, maze = cv.threshold(maze, 100, 255, cv.THRESH_BINARY)

    cv.imshow('map', maze)
    cv.waitKey(0)

    cv.imwrite("processed_maze.png", maze)

    cv.destroyAllWindows()

def process(image_path):
    black = np.zeros((3100, 3100), dtype=np.uint8)
    maze = cv.imread(image_path)

    cv.namedWindow('map', cv.WINDOW_NORMAL)
    cv.resizeWindow('map', 600, 600)
    
    thresh, maze = cv.threshold(maze, 140, 255, cv.THRESH_BINARY_INV)
    maze = cv.cvtColor(maze, cv.COLOR_BGR2GRAY)

    kernel = np.ones((71, 71), dtype = 'uint8')
    maze = cv.dilate(maze, kernel, iterations = 1)

    maze = cv.resize(maze, (300, 300), interpolation = 2)

    thresh, maze = cv.threshold(maze, 100, 255, cv.THRESH_BINARY)
    cv.imwrite("processed_maze.png", maze)


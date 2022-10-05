### =============================================== ###
# CONSTANTS

UNDEFINED = 0
UNPASSABLE = 1
OPEN = 2
CLOSED = 3
START = 4
PATH = 5

class_enum = ["Undefined ", "Unpassable", "Open      ", "Closed    ", "Start     ", "End       "] 

UNDEFINED_COLOR = [50, 50, 50]
UNPASSABLE_COLOR = [255, 255, 255]
CLOSED_COLOR = [77, 143, 84]
OPEN_COLOR = [34, 250, 248]
START_COLOR = [168, 109, 50]
PATH_COLOR = [163, 72, 227]

### =============================================== ###
# STRUCTURES

# Calculating an h cost for a single point
def gen_h(point, end):
    dx = abs(end[0] - point[0])
    dy = abs(end[1] - point[1])

    straight = abs(dx - dy)
    angle = min(dx, dy)

    return 10 * straight + 14 * angle

# Calculating a g costs of a point from a parent
def gen_g(child, parent, parent_g):
    if (child == parent):
        g = 0
    elif (child[0] == parent[0] or child[1] == parent[1]):
        g = 10
    else:
        g = 14
    
    return g + parent_g

### =============================================== ###
# TESTING SPACE

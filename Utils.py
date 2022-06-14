import math

TRUCK_SPEED = 40         
DRONE_SPEED = 50
ENDURANCE_OF_DRONE = 10
MAX_DISTANCE = 1000000
NO_OF_DRONE = 2
DELTA = 0.001

def printByLine(list):
    for ele in list:
        print(ele)

# Calculate distance between 2 points
def dist (x1, x2):
    return math.sqrt( ((x1[0] - x2[0])**2) + ((x1[1] - x2[1])**2) )

# Compare two float number
def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

# 2opt swap
def twoOptSwap(route, v1, v2):
    newRoute = []
    for i in range(0, v1 + 1):
        newRoute.append(route[i])
    for i in range(v2, v1, -1):
        newRoute.append(route[i])
    for i in range(v2 + 1, len(route)):
        newRoute.append(route[i])
    return newRoute
import math

TRUCK_SPEED = 40         
DRONE_SPEED = 50
ENDURANCE_OF_DRONE = 10
MAX_DISTANCE = 1000000
NO_OF_DRONE = 5
DELTA = 0.0001

def printByLine(list):
    for ele in list:
        print(ele)

# Calculate distance between 2 points
def dist (x1, x2):
    return math.sqrt( ((x1[0] - x2[0])**2) + ((x1[1] - x2[1])**2) )

# Compare two float number
def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)
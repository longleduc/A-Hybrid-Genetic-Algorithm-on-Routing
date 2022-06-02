totalTimeDroneEachAnchorPoint = [0 for i in range(noOfAnchorPoint)]
droneRoute = [[[] for j in range(NO_OF_DRONE)] for i in range(noOfAnchorPoint)]
# print("Rfinal in calculate fitness")
# print(Rfinal)
for i in range(noOfAnchorPoint):
    if (len(Rfinal[i]) > 0):
        for j in range(len(Rfinal[i])):
            min = -1
            for k in range(NO_OF_DRONE):
                if len(droneRoute[i][k]) == 0:
                    if min == -1:
                        min = k
                elif len(droneRoute[i][k]) < len(droneRoute[i][min]):
                    min = k
            droneRoute[i][min].append(copy.deepcopy(Rfinal[i][j]))

# print("DroneRoute")
# printByLine(droneRoute)
    
# Total time of drone 
for i in range(len(droneRoute)):
    # Each anchor point
    max = 0
    for j in range(len(droneRoute[i])):
        # Each drone
        totalDistance = 0
        if len(droneRoute[i][j]) > 0:
            for k in range(len(droneRoute[i][j])):
                totalDistance += droneRoute[i][j][k][1]
            if (totalDistance > max):
                max = totalDistance
    totalTimeDroneEachAnchorPoint[i] = max / DRONE_SPEED

totalTimeDrone = 0
for i in range(len(droneRoute)):
    totalTimeDrone += totalTimeDroneEachAnchorPoint[i]

distanceOfTruck = 0
for i in range(len(Dfinal)):
    if (i == 0):
        distanceOfTruck = dist((0, 0), coorOfAnchorPoints[Dfinal[i]])
    else:
        distanceOfTruck += dist(coorOfAnchorPoints[Dfinal[i - 1]], coorOfAnchorPoints[Dfinal[i]])
distanceOfTruck += dist(coorOfAnchorPoints[Dfinal[-1]], (0, 0))
totalTimeTruck = distanceOfTruck / TRUCK_SPEED

print(totalTimeDrone + totalTimeTruck)
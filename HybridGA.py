# HOW TO RUN THE CODE 
# python HybridGA.py "name of instance file" e.g: python HybridGA.py 100.10.1
from os import urandom
from re import L
from numpy.core.fromnumeric import sort
from Utils import *
import sys
from sklearn.cluster import KMeans
import numpy as np
import random
import copy

noOfCustomer = 0
noOfAnchorPoint = 0
coorOfCustomers = []
coorOfAnchorPoints = []
droneRoute = []

iterationMax = 0

#Read data from txt file
def readData():
    fileName = str(sys.argv[1])
    f = open("NewInstances/" + fileName + ".txt", "r")

    noOfCustomer = int(f.readline())

    coorOfCustomers = []
    for i in range(noOfCustomer):
        tmp = f.readline().split()
        coorOfCustomers.append([float(tmp[0]), float(tmp[1])])

    noOfAnchorPoint = int(f.readline())

    coorOfAnchorPoints = []
    for i in range(noOfAnchorPoint):
        tmp = f.readline().split()
        coorOfAnchorPoints.append([float(tmp[0]), float(tmp[1])])
    return noOfCustomer, np.array(coorOfCustomers), noOfAnchorPoint, np.array(coorOfAnchorPoints)

def calculateFitness(Dfinal, Rfinal):
    totalTimeDroneEachAnchorPoint = [0 for i in range(noOfAnchorPoint)]
    droneRoute = [[[] for j in range(NO_OF_DRONE)] for i in range(noOfAnchorPoint)]
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
                droneRoute[i][min].append(Rfinal[i][j])

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
    
    return totalTimeDrone + totalTimeTruck

def populationInitial():
    population = []
    iteration = 0
    while (iteration < iterationMax or len(population) < 3):
        iteration += 1
        print(iteration, len(population))
        Dtemp = []
        Ntemp = []
        Dfinal = []
        Nfinal = []
        for i in range(noOfAnchorPoint):
            Dtemp.append(i)
        for i in range(noOfCustomer):
            Ntemp.append(i)
        Dcover = []
        Ncover = [[] for i in range(noOfCustomer)]
        weightD = []
        for i in range(noOfAnchorPoint):
            Dcover.append([])
            for j in range(noOfCustomer):
                if (dist(coorOfAnchorPoints[Dtemp[i]], coorOfCustomers[Ntemp[j]]) <= ENDURANCE_OF_DRONE / 2):
                    Dcover[i].append(j)
                    Ncover[j].append(i)
        
        for i in range(noOfCustomer):
            if (len(Ncover[i]) == 1):
                dmust = Ncover[i][0]
                Dfinal.append(dmust)
                Dtemp.remove(dmust)
                for j in range(len(Dcover[dmust])):
                    Nfinal.append(Dcover[dmust][j])
                    Ntemp.remove(Dcover[dmust][j])

        while len(Ntemp) != 0:
            weightD = []
            for i in range(len(Dtemp)):
                count = 0

                for j in range(len(Dcover[Dtemp[i]])):
                    if (Ntemp.count(Dcover[Dtemp[i]][j]) > 0):
                        count += 1

                weightD.append(count)

            drandom = random.choices(Dtemp, weights=weightD, k=1)
            Nnum = []
            for i in range(len(Ntemp)):
                if (Dcover[drandom[0]].count(Ntemp[i]) > 0):
                    Nnum.append(Ntemp[i])

            if (len(Nnum) == 0):
                continue
                
            Dfinal.append(drandom[0])
            Dtemp.remove(drandom[0])
            for i in range(len(Nnum)):
                Nfinal.append(Nnum[i])
            for i in range(len(Nnum)):
                Ntemp.remove(Nnum[i])
        
        # print("Dfinal")
        # print(Dfinal)
        # print("Nfinal")
        # print(Nfinal)

        Dnearest = [[] for i in range(noOfAnchorPoint)]
        for i in range(noOfCustomer):
            min = MAX_DISTANCE
            nearestAnchorPoint = 0
            for j in range(len(Dfinal)):
                if (dist(coorOfCustomers[i], coorOfAnchorPoints[Dfinal[j]]) < min):
                    min = dist(coorOfCustomers[i], coorOfAnchorPoints[Dfinal[j]])
                    nearestAnchorPoint = j
            Dnearest[Dfinal[nearestAnchorPoint]].append((i, dist(coorOfCustomers[i], coorOfAnchorPoints[Dfinal[nearestAnchorPoint]])))      
        
        # print("Dnearest")
        # print(Dnearest)

        Rtemp = [[] for i in range(noOfAnchorPoint)]
        Rfinal = [[] for i in range(noOfAnchorPoint)]
        for i in range(len(Dfinal)):
            # Create route for drone
            currentAnchorPoint = Dfinal[i]
            Dnearest[currentAnchorPoint] = sorted(Dnearest[currentAnchorPoint], key=lambda x: (x[1]), reverse=True)
            currentRoute = [-currentAnchorPoint - 1]
            totalDistance = 0
            for j in range(len(Dnearest[currentAnchorPoint])):
                currentCustomer = Dnearest[currentAnchorPoint][j][0]
                if (currentRoute[-1] < 0):
                    totalDistance += dist(coorOfAnchorPoints[currentAnchorPoint], coorOfCustomers[currentCustomer])
                else:
                    totalDistance += dist(coorOfCustomers[currentRoute[-1]], coorOfCustomers[currentCustomer])
                if (totalDistance + dist(coorOfCustomers[currentCustomer], coorOfAnchorPoints[currentAnchorPoint]) <= ENDURANCE_OF_DRONE):
                    currentRoute.append(currentCustomer)
                    tempRoute = copy.deepcopy(currentRoute)
                    tempRoute.append(-currentAnchorPoint - 1)
                    Rtemp[currentAnchorPoint].append((tempRoute, totalDistance + dist(coorOfCustomers[currentCustomer], coorOfAnchorPoints[currentAnchorPoint])))
                else:
                    currentRoute = [-currentAnchorPoint - 1]
                    currentRoute.append(currentCustomer)
                    totalDistance = dist(coorOfAnchorPoints[currentAnchorPoint], coorOfCustomers[currentCustomer])
                    tempRoute = copy.deepcopy(currentRoute)
                    tempRoute.append(-currentAnchorPoint - 1)
                    Rtemp[currentAnchorPoint].append((tempRoute, totalDistance + dist(coorOfCustomers[currentCustomer], coorOfAnchorPoints[currentAnchorPoint])))

            Rtemp[currentAnchorPoint] = sorted(Rtemp[currentAnchorPoint], key=lambda x: (x[1]), reverse=True)
            passed = [False for j in range(noOfCustomer)]
            for j in range(len(Rtemp[currentAnchorPoint])):
                check = True

                for k in range(len(Rtemp[currentAnchorPoint][j][0])):
                    if Rtemp[currentAnchorPoint][j][0][k] > 0 and passed[Rtemp[currentAnchorPoint][j][0][k]]:
                        check = False

                if check:
                    Rfinal[currentAnchorPoint].append(Rtemp[currentAnchorPoint][j])

                for k in range(len(Rfinal[currentAnchorPoint][-1][0])):
                    if (Rfinal[currentAnchorPoint][-1][0][k] > 0):
                        passed[Rfinal[currentAnchorPoint][-1][0][k]] = True
        # print("Rfinal")
        # printByLine(Rfinal)        

        fitness = calculateFitness(copy.deepcopy(Dfinal), copy.deepcopy(Rfinal))  
        ok = True
        for i in range(len(population)):
            if (fitness - population[i][2] < DELTA):
                ok = False
        if (ok):
            population.append((Dfinal, Rfinal, fitness))
    return population

def crossOver():
    i, j = random.sample(range(0, len(population)), 2)
    if (population[i][2] > population[j][2]):
        P1 = i
    else:
        P1 = j
    
    P2 = P1
    while (True):
        i, j = random.sample(range(0, len(population)), 2)
        if (population[i][2] > population[j][2]):
            P2 = i
        else:
            P2 = j
        if (P2 != P1):
            break
    
    print(P1, P2)
    RP1 = population[P1][1]
    RP2 = population[P2][1]
    Rtotal = []
    for i in range(len(RP1)):
        if (len(RP1[i]) > 0):
            for j in range(len(RP1[i])):
                Rtotal.append(RP1[i][j])
    for i in range(len(RP2)):
        if (len(RP2[i]) > 0):
            for j in range(len(RP2[i])):
                Rtotal.append(RP2[i][j])
    print("Rtotal")
    printByLine(Rtotal)

    DP1 = population[P1][0]
    DP2 = population[P2][0]
    Dtotal = []
    for i in range(len(DP1)):
        if (Dtotal.count(DP1[i]) == 0):
            Dtotal.append(DP1[i])
    for i in range(len(DP2)):
        if (Dtotal.count(DP2[i]) == 0):
            Dtotal.append(DP2[i])
    
    print("Dtotal")
    print(Dtotal)

    Rfinal = []
    Dfinal = []
    while (len(Rtotal) != 0):
        Rtotal = sorted(Rtotal, key=lambda x: (x[1]), reverse=False)
        Rfinal.append(Rtotal[0])
        Rtotal.remove(Rfinal[-1])
        for i in range(1, len(Rfinal[-1][0]) - 1):
            deletedPoint = Rfinal[-1][0][i]
            for j in range(len(Rtotal)):
                for k in range(len(Rtotal[j][0])):
                    if (Rtotal[j][0][k] == deletedPoint):
                        # Calculate the distance again
                        tmp = list(Rtotal[j])
                        if (Rtotal[j][0][k - 1] < 0):
                            anchorPoint = abs(Rtotal[j][0][k - 1]) - 1
                            tmp[1] -= dist(coorOfAnchorPoints[anchorPoint], coorOfCustomers[Rtotal[j][0][k]])
                        else:
                            tmp[1] -= dist(coorOfCustomers[Rtotal[j][0][k - 1]], coorOfCustomers[Rtotal[j][0][k]])

                        if (Rtotal[j][0][k + 1] < 0):
                            anchorPoint = abs(Rtotal[j][0][k + 1]) - 1
                            tmp[1] -= dist(coorOfAnchorPoints[anchorPoint], coorOfCustomers[Rtotal[j][0][k]])
                        else:
                            tmp[1] -= dist(coorOfCustomers[Rtotal[j][0][k + 1]], coorOfCustomers[Rtotal[j][0][k]])
                        
                        if (Rtotal[j][0][k - 1] < 0):
                            anchorPoint = abs(Rtotal[j][0][k - 1]) - 1
                            if (Rtotal[j][0][k + 1] > 0):
                                tmp[1] += dist(coorOfAnchorPoints[anchorPoint], coorOfCustomers[Rtotal[j][0][k + 1]])
                        elif (Rtotal[j][0][k + 1] < 0):
                            anchorPoint = abs(Rtotal[j][0][k + 1]) - 1
                            if (Rtotal[j][0][k - 1] > 0):
                                tmp[1] += dist(coorOfAnchorPoints[anchorPoint], coorOfCustomers[Rtotal[j][0][k - 1]])
                        else:
                            tmp[1] += dist(coorOfCustomers[Rtotal[j][0][k - 1]], coorOfCustomers[Rtotal[j][0][k + 1]])
                                
                        Rtotal[j] = tuple(tmp) 
                        # Remove customer
                        Rtotal[j][0].remove(deletedPoint) 
                        break
            for route in Rtotal:
                if (len(route[0]) == 2):
                    Rtotal.remove(route)                    
    print("Rfinal")
    printByLine(Rfinal)
    print("Rtotal")
    printByLine(Rtotal)

    Rtemp = []
    while (len(Rfinal) > 0):
        weightD = [0 for i in range(len(Dtotal))]
        for i in range(len(Rfinal)):
            anchorPoint = abs(Rfinal[i][0][0]) - 1
            for j in range(len(Dtotal)):
                if Dtotal[j] == anchorPoint:
                    weightD[j] += 1
        drandom = random.choices(Dtotal, weights=weightD, k=1)[0]

        for route in Rfinal:
            anchorPoint = abs(route[0][0]) - 1
            if Dtotal[drandom] == anchorPoint:
                Rtemp.append(route)
                Rfinal.remove(route)
            else:
                # TODO: replace by this anchor point, if satisfy then add to Rtemp
                continue 
        break
    return 0

def education():
    return 0

def populationManagement():
    return 0

if __name__ == "__main__":
    noOfCustomer , coorOfCustomers, noOfAnchorPoint, coorOfAnchorPoints = readData()
    iterationMax = noOfCustomer
    # iterationMax = 2
    Route = [[] for i in range(noOfAnchorPoint)]

    population = populationInitial()
    print("Population")
    printByLine(population)
    crossOver()
    # for i in range(len(population)):
    #     crossOver()
    #     education()
    #     populationManagement()

    print("END")
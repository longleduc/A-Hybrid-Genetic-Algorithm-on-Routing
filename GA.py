# HOW TO RUN THE CODE 
# python HybridGA.py "name of instance file" e.g: python GA.py 100.10.1
from os import urandom
from re import L
from numpy.core.fromnumeric import sort
from Utils import *
import sys
from sklearn.cluster import KMeans
import numpy as np
import random
import copy
import HybridGA

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

def calculateRoute(route):
    totalDistance = 0
    for i in range(1, len(route)):
        if (i == 1):
            totalDistance += dist(coorOfAnchorPoints[abs(route[i - 1]) - 1], coorOfCustomers[route[i]])
        elif (i == len(route) - 1):
            totalDistance += dist(coorOfCustomers[route[i - 1]], coorOfAnchorPoints[abs(route[i]) - 1])
        else:
            totalDistance += dist(coorOfCustomers[route[i - 1]], coorOfCustomers[route[i]])
    return totalDistance

def calculateFitness(Dfinal, Rfinal):
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
    
    return totalTimeDrone + totalTimeTruck

def populationInitial():
    population = []
    iteration = 0
    while (iteration < iterationMax or len(population) < 3):
        iteration += 1
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
        # Record the set of anchor point cover by each customer
        # Record the set of customer cover by each anchor point
        for i in range(noOfAnchorPoint):
            Dcover.append([])
            for j in range(noOfCustomer):
                if (dist(coorOfAnchorPoints[Dtemp[i]], coorOfCustomers[Ntemp[j]]) <= ENDURANCE_OF_DRONE / 2):
                    Dcover[i].append(j)
                    Ncover[j].append(i)
        
        # Find customer cover by only one anchor point
        # That anchor point is dmust
        for i in range(noOfCustomer):
            if (len(Ncover[i]) == 1):
                dmust = copy.deepcopy(Ncover[i][0])
                Dfinal.append(copy.deepcopy(dmust))
                Dtemp.remove(dmust)
                for j in range(len(Dcover[dmust])):
                    Nfinal.append(copy.deepcopy(Dcover[dmust][j]))
                    Ntemp.remove(copy.deepcopy(Dcover[dmust][j]))
        
        # Random choose anchor points with weight
        # For each anchor point add all the customer it cover to Nfinal
        while len(Ntemp) != 0:
            weightD = []
            for i in range(len(Dtemp)):
                count = 0

                for j in range(len(Dcover[Dtemp[i]])):
                    if (Ntemp.count(Dcover[Dtemp[i]][j]) > 0):
                        count += 1

                weightD.append(copy.deepcopy(count))

            drandom = random.choices(Dtemp, weights=weightD, k=1)
            Nnum = []
            for i in range(len(Ntemp)):
                if (Dcover[drandom[0]].count(Ntemp[i]) > 0):
                    Nnum.append(copy.deepcopy(Ntemp[i]))

            if (len(Nnum) == 0):
                continue
                
            Dfinal.append(copy.deepcopy(drandom[0]))
            Dtemp.remove(copy.deepcopy(drandom[0]))
            for i in range(len(Nnum)):
                Nfinal.append(copy.deepcopy(Nnum[i]))
            for i in range(len(Nnum)):
                Ntemp.remove(copy.deepcopy(Nnum[i]))
        
        # print("Dfinal")
        # print(Dfinal)
        # print("Nfinal")
        # print(Nfinal)
        
        # Find the nearest anchor point for each customer
        # Each customer will be assign to that anchor point
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

        # Create route for drone
        # Rfinal[i] means all the route in anchor point i
        # Rfinal[i][j] means route j-th in anchor point i 
        # Each route is a tuple with first element is order of visit node, second element is the total distance travel
        Rtemp = [[] for i in range(noOfAnchorPoint)]
        Rfinal = [[] for i in range(noOfAnchorPoint)]
        for i in range(len(Dfinal)):
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
                    currentRoute.append(copy.deepcopy(currentCustomer))
                    tempRoute = copy.deepcopy(currentRoute)
                    tempRoute.append(-currentAnchorPoint - 1)
                    Rtemp[currentAnchorPoint].append((copy.deepcopy(tempRoute), totalDistance + dist(coorOfCustomers[currentCustomer], coorOfAnchorPoints[currentAnchorPoint])))
                else:
                    currentRoute = [-currentAnchorPoint - 1]
                    currentRoute.append(currentCustomer)
                    totalDistance = dist(coorOfAnchorPoints[currentAnchorPoint], coorOfCustomers[currentCustomer])
                    tempRoute = copy.deepcopy(currentRoute)
                    tempRoute.append(-currentAnchorPoint - 1)
                    Rtemp[currentAnchorPoint].append((copy.deepcopy(tempRoute), totalDistance + dist(coorOfCustomers[currentCustomer], coorOfAnchorPoints[currentAnchorPoint])))

            Rtemp[currentAnchorPoint] = sorted(Rtemp[currentAnchorPoint], key=lambda x: (x[1]), reverse=True)
            passed = [False for j in range(noOfCustomer)]
            for j in range(len(Rtemp[currentAnchorPoint])):
                check = True

                for k in range(len(Rtemp[currentAnchorPoint][j][0])):
                    if Rtemp[currentAnchorPoint][j][0][k] > 0 and passed[Rtemp[currentAnchorPoint][j][0][k]]:
                        check = False

                if check:
                    Rfinal[currentAnchorPoint].append(copy.deepcopy(Rtemp[currentAnchorPoint][j]))

                for k in range(len(Rfinal[currentAnchorPoint][-1][0])):
                    if (Rfinal[currentAnchorPoint][-1][0][k] > 0):
                        passed[Rfinal[currentAnchorPoint][-1][0][k]] = True
        # print("Rfinal")
        # printByLine(Rfinal)        

        fitness = calculateFitness(copy.deepcopy(Dfinal), copy.deepcopy(Rfinal))  
        population = populationManagement(copy.deepcopy(population), copy.deepcopy(Dfinal), copy.deepcopy(Rfinal), copy.deepcopy(fitness))
    return population

def populationManagement(population, Dfinal, Rfinal, fitness):
    ok = True
    for i in range(len(population)):
        if (abs(fitness - population[i][2]) < DELTA):
            ok = False
    if (ok):
        if (len(population) > noOfCustomer):
            population = sorted(population, key=lambda x: (x[2]), reverse=True)
            population.pop(0)
        population.append((copy.deepcopy(Dfinal), copy.deepcopy(Rfinal), copy.deepcopy(fitness)))
    return population

def calculateRouteFitness(customers):
    fitness = 0
    
    return fitness

def genetic_algorithm(customers):
    fitness = []

    # Insert random các số 0
    k = min (5, len(customers) - 1)
    zeroPos = random.sample(range(1, len(customers)), k)
    zeroPos = sorted(zeroPos)
    
    for i in range(len(zeroPos)):
        customers.insert(zeroPos[i] + i, -1)
    
    calculateRouteFitness(copy.deepcopy(customers))

    # for gen in range(iterationMax):
        # 
    return customers


def optimizeWithGA(population):
    Rtemp = copy.deepcopy(population[1])

    print("population")
    print(population)
    
    customersInAP = [[] for i in range(noOfAnchorPoint)]
    for routesInAnAnchorPoint in Rtemp:
        for route in routesInAnAnchorPoint:
            anchorPoint = 0
            for customer in route[0]:
                if (customer < 0):
                    anchorPoint = abs(customer) - 1
                else:
                    customersInAP[anchorPoint].append(copy.deepcopy(customer))

    for i in range(noOfAnchorPoint):
        if (len(customersInAP[i]) > 0):
            customersInAP[i] = genetic_algorithm(copy.deepcopy(customersInAP[i]))
    return 0

if __name__ == "__main__":
    noOfCustomer , coorOfCustomers, noOfAnchorPoint, coorOfAnchorPoints = readData()
    iterationMax = noOfCustomer
    # iterationMax = 2
    Route = [[] for i in range(noOfAnchorPoint)]

    population = populationInitial()
    # print("Population")
    # printByLine(population)

    for i in range(len(population)):
        optimizeWithGA(copy.deepcopy(population[i]))
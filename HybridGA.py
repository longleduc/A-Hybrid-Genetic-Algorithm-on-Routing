# HOW TO RUN THE CODE 
# python HybridGA.py "name of instance file" e.g: python HybridGA.py 100.10.1
from fileinput import filename
from os import urandom
from re import L
import statistics
from numpy.core.fromnumeric import sort
from Utils import *
import sys
from sklearn.cluster import KMeans
import numpy as np
import random
import copy
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from time import time

noOfCustomer = 0
noOfAnchorPoint = 0
coorOfCustomers = []
coorOfAnchorPoints = []
droneRoute = []
bestSolution = []
markCrossOver = []
solutionEachStep = []

populationNumber = 0
populationSize = 0
iterationMax = 0

#Read data from txt file
def readData():
    fileName = str(sys.argv[1])
    f = open("NewInstancesCell/" + fileName + ".txt", "r")

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

def createDroneRoute(Rfinal):
    droneRoute = [[[] for j in range(NO_OF_DRONE)] for i in range(noOfAnchorPoint)]
    # print("Rfinal in calculate fitness")
    # print(Rfinal)
    for i in range(noOfAnchorPoint):
        if (len(Rfinal[i]) > 0):
            totalDistanceOfDrone = [0 for i in range(NO_OF_DRONE)]
            for j in range(len(Rfinal[i])):
                minTotalDistance = MAX_DISTANCE
                min = -1
                for k in range(NO_OF_DRONE):
                    if totalDistanceOfDrone[k] < minTotalDistance:
                        min = copy.deepcopy(k)
                        minTotalDistance = copy.deepcopy(totalDistanceOfDrone[k])
                droneRoute[i][min].append(copy.deepcopy(Rfinal[i][j]))
                totalDistanceOfDrone[min] += Rfinal[i][j][1]
    return droneRoute
    
def prepareRouteForFitness(Rtemp):
    Rfinal = [[] for i in range(noOfAnchorPoint)]
    for route in Rtemp:
        anchorPoint = abs(route[0][0]) - 1
        Rfinal[anchorPoint].append(copy.deepcopy(route))
    return Rfinal

def calculateRoute(route):
    totalDistance = 0
    if (len(route) == 2):
        return 0  
    for i in range(1, len(route)):
        if (i == 1):
            totalDistance += dist(coorOfAnchorPoints[abs(route[i - 1]) - 1], coorOfCustomers[route[i]])
        elif (i == len(route) - 1):
            totalDistance += dist(coorOfCustomers[route[i - 1]], coorOfAnchorPoints[abs(route[i]) - 1])
        else:
            totalDistance += dist(coorOfCustomers[route[i - 1]], coorOfCustomers[route[i]])
    return totalDistance

def calculateRouteAnchorPoint(D):
    totalDistance = 0
    for i in range(0, len(D)):
        if (i == 0):
            totalDistance += dist((0, 0), coorOfAnchorPoints[D[i]])
        else:
            totalDistance += dist(coorOfAnchorPoints[D[i - 1]], coorOfAnchorPoints[D[i]])
    return totalDistance

def calculateFitness(Dfinal, Rfinal):
    totalTimeDroneEachAnchorPoint = [0 for i in range(noOfAnchorPoint)]
    # droneRoute = [[[] for j in range(NO_OF_DRONE)] for i in range(noOfAnchorPoint)]
    # # print("Rfinal in calculate fitness")
    # # print(Rfinal)
    # for i in range(noOfAnchorPoint):
    #     if (len(Rfinal[i]) > 0):
    #         totalDistanceOfDrone = [0 for i in range(NO_OF_DRONE)]
    #         minTotalDistance = MAX_DISTANCE
    #         for j in range(len(Rfinal[i])):
    #             min = -1
    #             for k in range(NO_OF_DRONE):
    #                 if len(droneRoute[i][k]) == 0:
    #                     min = k
    #                     break
    #                 elif totalDistanceOfDrone[k] < minTotalDistance:
    #                     min = k
    #                     minTotalDistance = totalDistanceOfDrone[k]
    #             droneRoute[i][min].append(copy.deepcopy(Rfinal[i][j]))
    #             totalDistanceOfDrone[min] += Rfinal[i][j][1]
    droneRoute = createDroneRoute(copy.deepcopy(Rfinal))

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

    # Tính đường đi cho truck
    distanceOfTruck = 0
    for i in range(len(Dfinal)):
        if (i == 0):
            distanceOfTruck = dist((0, 0), coorOfAnchorPoints[Dfinal[i]])
        else:
            distanceOfTruck += dist(coorOfAnchorPoints[Dfinal[i - 1]], coorOfAnchorPoints[Dfinal[i]])
    distanceOfTruck += dist(coorOfAnchorPoints[Dfinal[-1]], (0, 0))
    # totalTimeTruck = distanceOfTruck / TRUCK_SPEED
    # Sử dụng 2-opt để tối ưu đường đi cho truck
    improved = True
    Dtemp = copy.deepcopy(Dfinal)
    bestDistance = distanceOfTruck
    while(improved):
        # bestDistance = distanceOfTruck
        improved = False
        for i in range (1, len(Dtemp) - 2):
            if (not improved):
                for j in range(i + 2, len(Dtemp) - 1):
                    newDtemp = twoOptSwap(copy.deepcopy(Dtemp), i, j)
                    newDistance = calculateRouteAnchorPoint(newDtemp)
                    if (newDistance < bestDistance):
                        improved = True
                        Dtemp = newDtemp
                        bestDistance = newDistance
                        break
    totalTimeTruck = bestDistance / TRUCK_SPEED
    return totalTimeDrone + totalTimeTruck

def binaryTournament(population):
    i, j = random.sample(range(0, len(population)), 2)
    if (population[i][2] > population[j][2]):
        return i
    return j

def populationInitial():
    population = []
    iteration = 0
    while (len(population) == populationSize or iteration < iterationMax or len(population) < 3):
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
                if (Dtemp.count(dmust) > 0):
                    Dfinal.append(copy.deepcopy(dmust))
                    Dtemp.remove(dmust)
                    for j in range(len(Dcover[dmust])):
                        if (Nfinal.count(copy.deepcopy(Dcover[dmust][j])) == 0):
                            Nfinal.append(copy.deepcopy(Dcover[dmust][j]))
                        if (Ntemp.count(Dcover[dmust][j]) > 0):
                            Ntemp.remove(copy.deepcopy(Dcover[dmust][j]))
        # print("Dfinal")
        # print(Dfinal)
        # print("Nfinal")
        # Nfinal.sort()
        # print(Nfinal)

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
            # TODO: chon theo banh xe roulet
            # https://www.baeldung.com/cs/genetic-algorithms-roulette-selection
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
            # TODO: đi ra xa nhất, rồi từ điểm thăm hiện tại, tiếp tục đi tới điểm gần hiện tại nhất 
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

            #TODO: xem lại
            Rtemp[currentAnchorPoint] = sorted(Rtemp[currentAnchorPoint], key=lambda x: (x[1]), reverse=True)
            passed = [False for j in range(noOfCustomer)]
            for j in range(len(Rtemp[currentAnchorPoint])):
                check = True

                for k in range(len(Rtemp[currentAnchorPoint][j][0])):
                    if Rtemp[currentAnchorPoint][j][0][k] >= 0 and passed[Rtemp[currentAnchorPoint][j][0][k]]:
                        check = False

                if check:
                    Rfinal[currentAnchorPoint].append(copy.deepcopy(Rtemp[currentAnchorPoint][j]))

                for k in range(len(Rfinal[currentAnchorPoint][-1][0])):
                    if (Rfinal[currentAnchorPoint][-1][0][k] >= 0):
                        passed[Rfinal[currentAnchorPoint][-1][0][k]] = True
        # print("Rfinal")
        # printByLine(Rfinal)        

        fitness = calculateFitness(copy.deepcopy(Dfinal), copy.deepcopy(Rfinal))  
        population = populationManagement(copy.deepcopy(population), copy.deepcopy(Dfinal), copy.deepcopy(Rfinal), copy.deepcopy(fitness))
    return population

def crossOver():
    # Binary tournament to choose Parent 1 and Parent 2 for cross over
    while (True):
        P1 = binaryTournament(copy.deepcopy(population))
        while (True):
            P2 = binaryTournament(copy.deepcopy(population))
            if (P2 != P1):
                break
        index1 = population[P1][3]
        index2 = population[P2][3]
        if (markCrossOver.count((index1, index2)) == 0 and markCrossOver.count((index2, index1)) == 0):
            markCrossOver.append((index1, index2))
            markCrossOver.append((index2, index1))
            break
        # print("NO CROSS AGAIN!")
        
    RP1 = copy.deepcopy(population[P1][1])
    RP2 = copy.deepcopy(population[P2][1])
    # Extract all route in P1 and P2 and add to Rtotal
    Rtotal = []
    for i in range(len(RP1)):
        if (len(RP1[i]) > 0):
            for j in range(len(RP1[i])):
                Rtotal.append(RP1[i][j])
    for i in range(len(RP2)):
        if (len(RP2[i]) > 0):
            for j in range(len(RP2[i])):
                Rtotal.append(RP2[i][j])
    # Extract all anchor point in P1 and P2 and add to Dtotal
    DP1 = copy.deepcopy(population[P1][0])
    DP2 = copy.deepcopy(population[P2][0])
    Dtotal = []
    for i in range(len(DP1)):
        if (Dtotal.count(DP1[i]) == 0):
            Dtotal.append(DP1[i])
    for i in range(len(DP2)):
        if (Dtotal.count(DP2[i]) == 0):
            Dtotal.append(DP2[i])
    # Select the minimum visit cost route in Rtotal
    # All nodes in the route we select will be deleted from Rtotal
    Rfinal = []
    while (len(Rtotal) != 0):
        Rtotal = sorted(Rtotal, key=lambda x: (x[1]), reverse=False)
        Rfinal.append(copy.deepcopy(Rtotal[0]))
        Rtotal.remove(Rfinal[-1])
        for i in range(len(Rfinal[-1][0])):
            if (Rfinal[-1][0][i] >= 0):
                deletedPoint = Rfinal[-1][0][i]
                for j in range(len(Rtotal)):
                    for k in range(len(Rtotal[j][0])):
                        if (Rtotal[j][0][k] == deletedPoint):
                            # Remove customer
                            Rtotal[j][0].remove(deletedPoint) 
                            # Calculate distance again
                            distance = calculateRoute(Rtotal[j][0])
                            Rtotal[j] = (Rtotal[j][0], distance)
                            break
                for route in Rtotal:
                    if (len(route[0]) == 2):
                        Rtotal.remove(route)        
    # print(population[P1])
    # print(population[P2])
    # Select anchor point with min weight and all route it cover and add to Dfinal and Rfinal
    # weight of an anchor point = number of route connect to it 
    Dfinal = []
    Rtemp = []
    while (len(Rfinal) > 0):
        weightD = [0 for i in range(len(Dtotal))]
        for i in range(len(Rfinal)):
            anchorPoint = abs(Rfinal[i][0][0]) - 1
            for j in range(len(Dtotal)):
                if Dtotal[j] == anchorPoint:
                    weightD[j] += 1
        maxWeight = -1
        for i in range(len(Dtotal)):
            if (weightD[i] > maxWeight):
                maxWeight = copy.deepcopy(weightD[i])
                drandom = copy.deepcopy(Dtotal[i])
        # drandom = random.choices(Dtotal, weights=weightD, k=1)[0]   
        Dfinal.append(copy.deepcopy(drandom))
        
        # print("Rfinal")
        # print(Rfinal)
        # print("Dtotal")
        # print(Dtotal)
        # print("weightD")
        # print(weightD)
        # print("\n")

        Rdelete = []
        for route in Rfinal:
            anchorPoint = abs(route[0][0]) - 1
            if drandom == anchorPoint:
                Rtemp.append(copy.deepcopy(route))
                Rdelete.append(copy.deepcopy(route))
            else:
                routeTmp = copy.deepcopy(route)
                routeTmp[0][0] = -drandom - 1
                routeTmp[0][-1] = -drandom - 1
                tmp = tuple((routeTmp[0], calculateRoute(routeTmp[0])))
                routeTmp = tmp
                if (routeTmp[1] < ENDURANCE_OF_DRONE):
                    Rtemp.append(copy.deepcopy(routeTmp))
                    Rdelete.append(copy.deepcopy(route))
        
        for route in Rdelete:
            Rfinal.remove(route)
        # Dtotal.remove(drandom)
    # print("Crossover Result:")
    # print("Dfinal")
    # print(Dfinal)
    # print("Rtemp")
    # print(Rtemp)
    return Dfinal,Rtemp

def education(Dfinal, Rfinal):
    # ANCHOR POINT BASED EDUCATION
    Ddelete = []
    Dtemp = copy.deepcopy(Dfinal)
    numRouteInAnchorPoint = [0 for i in range(noOfAnchorPoint)]
    for route in Rfinal:
        anchorPoint = abs(route[0][0]) - 1
        numRouteInAnchorPoint[anchorPoint] += 1
    tmpArr = []
    for anchorPoint in Dtemp:
        tmpArr.append(copy.deepcopy(numRouteInAnchorPoint[anchorPoint]))
    MIN_ROUTE_ANCHOR_POINT = round(np.median(tmpArr)) + 1
    # Select the anchor that have number of route < MIN_ROUTE_ANCHOR_POINT
    # Try to move all the route in that anchor point to another anchor point
    # If move completed, delete that anchor point
    for d in Dtemp:
        if (numRouteInAnchorPoint[d] < MIN_ROUTE_ANCHOR_POINT):
            Rtemp = copy.deepcopy(Rfinal)
            # Changable đảm bảo tất cả các cạnh trong anchor point đều có thể gán sang anchor point khác
            changable = True
            index = 0
            Dmore = []
            for route in Rfinal:
                anchorPoint = abs(route[0][0]) - 1
                if (changable and anchorPoint == d):
                    # Check để kiểm tra xem 1 anchor point có thể được gán sang anchor point khác hay không
                    check = False
                    for i in range(noOfAnchorPoint):
                        if (i != anchorPoint):
                            routeTmp = copy.deepcopy(route)
                            routeTmp[0][0] = -i - 1
                            routeTmp[0][-1] = -i - 1
                            tmp = tuple((routeTmp[0], calculateRoute(routeTmp[0])))
                            routeTmp = tmp
                            if (routeTmp[1] < ENDURANCE_OF_DRONE):
                                check = True
                                Rtemp[index] = routeTmp
                                # Nếu là 1 anchor point mới thì add vào Dmore để thêm vào Dtemp sau
                                if Dtemp.count(i) == 0 and Dmore.count(i) == 0:
                                    Dmore.append(copy.deepcopy(i))
                                break
                    if (not check):
                        changable = False
                        break
                index += 1
            if (changable):
                Rfinal = Rtemp
                for anchorPoint in Dmore:
                    Dtemp.append(anchorPoint)        
    Dtemp = []
    for route in Rfinal:
        anchorPoint = abs(route[0][0]) - 1
        if (Dtemp.count(anchorPoint) == 0):
            Dtemp.append(anchorPoint)
    
    # print("Anchor point based education result: ")
    # print("Dtemp")
    # print(Dtemp)
    # print("Rfinal")
    # print(Rfinal)
    # ROUTE BASED EDUCATION
    # Select all short route which have total distance < SHORT_DIS
    currentFitness = calculateFitness(Dtemp, prepareRouteForFitness(copy.deepcopy(Rfinal)))
    tmpArr = []
    for route in Rfinal:
        tmpArr.append(copy.deepcopy(route[1]))
    SHORT_DIS = np.median(tmpArr) + 1
    Rshort = []
    for route in Rfinal:
        if (route[1] < SHORT_DIS):
            Rshort.append(copy.deepcopy(route))
    for route in Rshort:
        Rfinal.remove(route)
    
    # Choose a random pair index1, index2 and try to merge them into a new route which still satisfy the distance constraint and the final result is smaller
    maxIteration = noOfCustomer
    iteration = 0
    # print(Dtemp)
    # print(Rfinal)
    # print(currentFitness)
    while (len(Rshort) > 1 and iteration < maxIteration):
        Rtemp = copy.deepcopy(Rfinal)
        index1 = 0
        index2 = 0
        while (index1 == index2):
            index1, index2 = random.sample(range(0, len(Rshort)), 2)   
        route1 = copy.deepcopy(Rshort[index1])
        route2 = copy.deepcopy(Rshort[index2])
        anchorPoint1 = abs(route1[0][0]) - 1
        anchorPoint2 = abs(route2[0][0]) - 1
        newRoute = []
        for i in range(len(route1[0])):
            if (route1[0][i] >= 0):
                newRoute.append(copy.deepcopy(route1[0][i]))
        for i in range(len(route2[0])):
            if (route2[0][i] >= 0):
                newRoute.append(copy.deepcopy(route2[0][i]))
        # Thử với từng anchorpoint thứ 1
        newRoute.insert(0, -anchorPoint1 - 1)
        newRoute.append(-anchorPoint1 - 1)
        distance = calculateRoute(newRoute)
        if (distance < ENDURANCE_OF_DRONE):
            Rtemp.append((copy.deepcopy(newRoute), distance))
            for route in Rshort:
                if (route != route1 and route != route2):
                    Rtemp.append(copy.deepcopy(route))
            fitness = calculateFitness(Dtemp, prepareRouteForFitness(copy.deepcopy(Rtemp)))
            if (fitness < currentFitness):
                Rfinal.append((copy.deepcopy(newRoute), distance))
                tmp1 = Rshort[index1]
                tmp2 = Rshort[index2]
                Rshort.remove(tmp1)
                Rshort.remove(tmp2)
                currentFitness = fitness
        elif (anchorPoint2 != anchorPoint1):
            # Thử với anchorpoint thứ 2 
            newRoute[0] = -anchorPoint2 - 1
            newRoute[-1] = -anchorPoint2 - 1
            distance = calculateRoute(newRoute)
            if (distance < ENDURANCE_OF_DRONE):
                Rtemp.append((copy.deepcopy(newRoute), distance))
                for route in Rshort:
                    if (route != route1 and route != route2):
                        Rtemp.append(copy.deepcopy(route))
                fitness = calculateFitness(Dtemp, prepareRouteForFitness(copy.deepcopy(Rtemp)))
                if (fitness < currentFitness):
                    Rfinal.append((copy.deepcopy(newRoute), distance))
                    tmp1 = Rshort[index1]
                    tmp2 = Rshort[index2]
                    Rshort.remove(tmp1)
                    Rshort.remove(tmp2)
                    currentFitness = fitness
        iteration += 1
    for route in Rshort:
        Rfinal.append(copy.deepcopy(route))
    Rshort = []

    # print("Path based education result: ")
    # print("Dtemp")
    # print(Dtemp)
    # print("Rfinal")
    # print(Rfinal)
    
    # CUSTOMER BASED EDUCATION
    # Using 2-opt
    for route in Rfinal:
        if (len(route[0]) > 3):
            improved = True
            while(improved):
                bestDistance = route[1]
                improved = False
                for i in range (1, len(route[0]) - 2):
                    if (not improved):
                        for j in range(i + 2, len(route[0]) - 1):
                            newRoute = twoOptSwap(copy.deepcopy(route[0]), i, j)
                            newDistance = calculateRoute(newRoute)
                            if (newDistance < bestDistance):
                                improved = True
                                route = (newRoute, newDistance)
                                bestDistance = newDistance
                                break
    # Exchange in each route
    for route in Rfinal:
        if (len(route[0]) > 3):
            for i in range(len(route[0])):
                if (route[0][i] >= 0):
                    newRoute = copy.deepcopy(route[0])
                    for j in range(len(route[0])):
                        if (route[0][j] >= 0):
                            tmp = newRoute[i]
                            newRoute[i] = newRoute[j]
                            newRoute[j] = tmp
                            newDistance = calculateRoute(newRoute)
                            if (newDistance < route[0][1]):
                                route = (newRoute, newDistance)
                                break
    # Replace in each route
    for route in Rfinal:
        if (len(route[0]) > 3):
            for i in range(len(route[0])):
                if (route[0][i] >= 0):
                    newRoute = copy.deepcopy(route[0])
                    for j in range(len(route[0])):
                        if j > 0 and j < len(route[0]) - 1 and  j != i:
                            newRoute.pop(i)
                            newRoute.insert(j, route[0][i])
                            newDistance = calculateRoute(newRoute)
                            if (newDistance < route[0][1]):
                                route = (newRoute, newDistance)
                                break
    
    # Exchange in two neighboring route
    for i in range(len(Rfinal)):
        route1 = copy.deepcopy(Rfinal[i][0])
        fitnessRoute1 = Rfinal[i][1]
        for j in range(len(Rfinal)):
            if (i != j):
                route2 = copy.deepcopy(Rfinal[j][0])
                fitnessRoute2 = Rfinal[j][1]
                for index1 in range(len(route1)):
                    customer1 = copy.deepcopy(route1[index1])
                    if (customer1 >= 0):
                        for index2 in range(len(route2)):
                            customer2 = copy.deepcopy(route2[index2])
                            if (customer2 >= 0):
                                newRoute1 = copy.deepcopy(route1)
                                newRoute2 = copy.deepcopy(route2)
                                newRoute1[index1] = customer2
                                newRoute2[index2] = customer1
                                newFitnessRoute1 = calculateRoute(newRoute1)
                                newFitnessRoute2 = calculateRoute(newRoute2)
                                if (newFitnessRoute1 < ENDURANCE_OF_DRONE 
                                and newFitnessRoute2 < ENDURANCE_OF_DRONE 
                                and newFitnessRoute1 + newFitnessRoute2 < fitnessRoute1 + fitnessRoute2):
                                    Rfinal[i] = (newRoute1, newFitnessRoute1)
                                    Rfinal[j] = (newRoute2, newFitnessRoute2)
                                    # print("SWAP TWO NEIGHBOR ROUTE:")
                                    # print(route1)
                                    # print(route2)
                                    # print(newRoute1)
                                    # print(newRoute2)
                                    route1 = newRoute1
                                    route2 = newRoute2
                                    fitnessRoute1 = newFitnessRoute1
                                    fitnessRoute2 = newFitnessRoute2
                                    customer1 = route1[index1]
                                    customer2 = route2[index2]
                                    
    # Replace between two neighboring route
    # for i in range(len(Rfinal)):
    #     route1 = copy.deepcopy(Rfinal[i][0])
    #     fitnessRoute1 = Rfinal[i][1]
    #     for j in range(len(Rfinal)):
    #         if (i != j):
    #             route2 = copy.deepcopy(Rfinal[j][0])
    #             fitnessRoute2 = Rfinal[j][1]
    #             for index1 in range(len(route1)):
    #                 customer1 = copy.deepcopy(route1[index1])
    #                 if (customer1 >= 0):
    #                     for index2 in range(len(route2)):
    #                         if (index2 > 0 and index2 < len(route2) - 1):
    #                             newRoute1 = copy.deepcopy(route1)
    #                             newRoute2 = copy.deepcopy(route2)
    #                             newRoute1.remove(customer1)
    #                             newRoute2.insert(index2, customer1)
    #                             newFitnessRoute1 = calculateRoute(newRoute1)
    #                             newFitnessRoute2 = calculateRoute(newRoute2)
    #                             if (newFitnessRoute1 < ENDURANCE_OF_DRONE 
    #                             and newFitnessRoute2 < ENDURANCE_OF_DRONE 
    #                             and newFitnessRoute1 + newFitnessRoute2 < fitnessRoute1 + fitnessRoute2):
    #                                 Rfinal[i] = (newRoute1, newFitnessRoute1)
    #                                 Rfinal[j] = (newRoute2, newFitnessRoute2)
    #                                 # print("SWAP TWO NEIGHBOR ROUTE:")
    #                                 # print(route1)
    #                                 # print(route2)
    #                                 # print(newRoute1)
    #                                 # print(newRoute2)
    #                                 route1 = newRoute1
    #                                 route2 = newRoute2
    #                                 fitnessRoute1 = newFitnessRoute1
    #                                 fitnessRoute2 = newFitnessRoute2
    #                                 customer1 = -1
    #                                 break

    # print("Customer based education result: ")
    # print("Dtemp")
    # print(Dtemp)
    # print("Rfinal")
    # print(Rfinal)
    # Create Rfinal again for calculate fitness
    Rtemp = Rfinal
    Rfinal = [[] for i in range(noOfAnchorPoint)]
    for route in Rtemp:
        anchorPoint = abs(route[0][0]) - 1
        Rfinal[anchorPoint].append(copy.deepcopy(route))
    
    # print("Education result:")
    # print("Dtemp")
    # print(Dtemp)
    # print("Rfinal")
    # print(Rfinal)
    return Dtemp, Rfinal, calculateFitness(copy.deepcopy(Dtemp), copy.deepcopy(Rfinal))

def populationManagement(population, Dfinal, Rfinal, fitness):
    global populationNumber
    ok = True
    for i in range(len(population)):
        if (abs(fitness - population[i][2]) < DELTA):
            ok = False
    if (ok):
        if (len(population) > noOfCustomer):
            population = sorted(population, key=lambda x: (x[2]), reverse=True)
            population.pop(0)
        populationNumber += 1
        population.append((copy.deepcopy(Dfinal), copy.deepcopy(Rfinal), copy.deepcopy(fitness), populationNumber))
    return population

if __name__ == "__main__":
    t0 = time()
    noOfCustomer , coorOfCustomers, noOfAnchorPoint, coorOfAnchorPoints = readData()
    iterationMax = noOfCustomer
    populationSize = 20
    # iterationMax = 2
    Route = [[] for i in range(noOfAnchorPoint)]

    population = populationInitial()
    print("Population")
    printByLine(population)
    print("\n")

    for i in range(1000):
        # print("Population in step ", i)
        Dfinal, Rfinal = crossOver()
        # print("Dfinal")
        # print(Dfinal)
        # print("Rfinal")
        # print(Rfinal)
        Dfinal, Rfinal, fitness = education(copy.deepcopy(Dfinal), copy.deepcopy(Rfinal))
        print("Dfinal in step ", i + 1)
        print(Dfinal)
        print("Rfinal in step ", i + 1)
        print(Rfinal)
        droneRoute = createDroneRoute(copy.deepcopy(Rfinal))    
        print("DroneRoute")
        printByLine(droneRoute)
        print("Fitness in step ", i + 1)
        print(fitness)
        print("\n")
        population = populationManagement(copy.deepcopy(population), copy.deepcopy(Dfinal), copy.deepcopy(Rfinal), copy.deepcopy(fitness))
        population = sorted(population, key=lambda x: (x[2]), reverse=True)
        bestSolution.append(population[-1][2])
        solutionEachStep.append(fitness)
        # print("Best solution in step ", i)
        # print(population[-1][2])
        # print("population")
        # printByLine(population)

    population = sorted(population, key=lambda x: (x[2]), reverse=True)
    print("Final solution")
    for i in range(len(bestSolution)):
        print("Best solution in step ", i, ": ", bestSolution[i])

    xAxis = [i for i in range(len(bestSolution))]
    t1 = time()
    plt.plot(xAxis[::20],solutionEachStep[::20])
    plt.xlabel('Number of iteration')
    plt.ylabel('Total time cost')
    plt.title(str(sys.argv[1]))
    plt.savefig('./Result Images/' + str(sys.argv[1]) + '.png')
    plt.show()
    
    print(f"{t1-t0}")
    print("END")
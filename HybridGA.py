# HOW TO RUN THE CODE 
# python HybridGA.py "name of instance file" e.g: python HybridGA.py 100.10.1
from os import urandom
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

def populationInitial():
    population = []
    Dfinal = []
    Nfinal = []
    for i in range(iterationMax):
        Dtemp = []
        Ntemp = []
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
        
        print(Dfinal)
        print(Nfinal)

        Dnearest = []
        for i in range(noOfCustomer):
            min = MAX_DISTANCE
            nearestAnchorPoint = 0
            for j in range(len(Dfinal)):
                if (dist(coorOfCustomers[i], coorOfAnchorPoints[Dfinal[j]]) < min):
                    min = dist(coorOfCustomers[i], coorOfAnchorPoints[Dfinal[j]])
                    nearestAnchorPoint = j
            Dnearest[Dfinal[nearestAnchorPoint]].append(i)      
        while (len(Dfinal) > 0 and len(Nfinal) > 0):
            d = Dfinal[i]
            for i in range(len(Dcover[d])):
                n = Dcover[d][i]
                    # TODO: Create path for drone 
                
    return 0

def crossOver():
    return 0

def educate():
    return 0

def populationManagement():
    return 0

if __name__ == "__main__":
    noOfCustomer , coorOfCustomers, noOfAnchorPoint, coorOfAnchorPoints = readData()
    # iterationMax = noOfCustomer
    iterationMax = 1
    
    population = populationInitial()
    for i in range(iterationMax):
        crossOver()
        educate()
        populationManagement()

    print("END")
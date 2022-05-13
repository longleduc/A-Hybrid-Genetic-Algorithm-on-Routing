# HOW TO RUN THE CODE 
# python HybridGA.py "name of instance file" e.g: python HybridGA.py 
from os import urandom
from numpy.core.fromnumeric import sort
from Utils import *
import sys
from sklearn.cluster import KMeans
import numpy as np
import random
import copy

0000000000000000000.0

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

if __name__ == "__main__":


    print("END")
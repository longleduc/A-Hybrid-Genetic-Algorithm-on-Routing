from fileinput import filename
from os import urandom
from turtle import circle
from numpy.core.fromnumeric import sort
from Utils import *
import sys
from sklearn.cluster import KMeans
import numpy as np
import random
import copy
import matplotlib.pyplot as plt

coordinators = [(-3, 3), (0, 3), (3, 3), (3, 0), (3, -3), (0, -3), (-3, -3), (-3, 0)]
coorOfAnchorPoint = []
def genData():
    fileName = str(sys.argv[1])
    f = open("Instances/" + fileName + ".txt", "r")

    noOfCustomer = int(f.readline().split()[1])

    f.readline()

    coorOfCustomer = []
    demands = []
    for i in range(noOfCustomer):
        tmp = f.readline().split()
        coorOfCustomer.append([float(tmp[0]), float(tmp[1])])
        demands.append(float(tmp[2]))

    f1 = open("NewInstancesCell/" + fileName + ".txt", "w+")
    f1.write(str(noOfCustomer) + "\n")
    for i in range(noOfCustomer):
        f1.write(str(coorOfCustomer[i][0]) + " " + str(coorOfCustomer[i][1]) + " " + str(demands[i]) + "\n")
    splitArr = fileName.split(".")
    noOfAnchorPoint = int(splitArr[1]) // 6 * 8
    numberOfIteration = int(splitArr[1]) // 6
    f1.writelines(str(noOfAnchorPoint) + "\n")
    for i in range(numberOfIteration):
        for coordinator in coordinators:
            f1.writelines(str(coordinator[0] * (i + 1)) + " " + str(coordinator[1] * (i + 1)) + "\n")
            coorOfAnchorPoint.append([coordinator[0] * (i + 1), coordinator[1] * (i + 1)])
    fig, ax = plt.subplots()
    customerX, customerY = map(list, zip(*coorOfCustomer))
    anchorX, anchorY = map(list, zip(*coorOfAnchorPoint))

    for x, y in coorOfAnchorPoint:
        ax.add_patch(plt.Circle((x,y),5,color='black', fill=False))

    ax.set_aspect('equal')
    ax.scatter(customerX, customerY, c='#008DC0', s=20)
    ax.scatter(anchorX, anchorY, c='black', s=80)
    ax.axhline(y=0, color='k')
    ax.axvline(x=0, color='k')
    plt.show()
    f.close()
    f1.close()

if __name__ == "__main__":
    genData()
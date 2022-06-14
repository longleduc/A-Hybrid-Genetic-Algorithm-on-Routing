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

def genData(noOfAnchorPoint):
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

    f1 = open("NewInstances/" + fileName + ".txt", "w+")
    f1.write(str(noOfCustomer) + "\n")
    for i in range(noOfCustomer):
        f1.write(str(coorOfCustomer[i][0]) + " " + str(coorOfCustomer[i][1]) + " " + str(demands[i]) + "\n")
    f1.writelines(str(noOfAnchorPoint) + "\n")
    kmeans = KMeans(n_clusters=noOfAnchorPoint, random_state=0).fit(coorOfCustomer)
    centers = kmeans.cluster_centers_
    for i in range(noOfAnchorPoint):
        f1.writelines(str(centers[i][0]) + " " + str(centers[i][1]) + "\n")

    # Draw data
    y_kmeans = kmeans.predict(coorOfCustomer)
    x = []
    y = []
    for i in range (noOfCustomer):
        x.append(coorOfCustomer[i][0])
        y.append(coorOfCustomer[i][1])
    plt.axis([0, 20, 0, 20])
    plt.axis("equal")
    plt.scatter(x, y, c=y_kmeans, s=50, cmap='viridis')
    plt.scatter(centers[:, 0], centers[:, 1], c='black', s=100, alpha=0.5)

    # Define circle
    circles = []
    for i in range(len(centers)):
        circles.append(plt.Circle(centers[i], 5, fill = False))
    for i in range(len(circles)):
        plt.gca().add_artist(circles[i])

    plt.xlim(-12, 12)
    plt.ylim(-12, 12)
    plt.show()

if __name__ == "__main__":
    genData(10)
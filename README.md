# A-Hybrid-Genetic-Algorithm-on-Routing
Thesis

How to run the code:
1) pip install -r requirements.txt
2) To choose which type of dataset to run:
Line 34 in readData() function: f = open("folder name of dataset/" + fileName + ".txt", "r")
E.x: f = open("NewInstances/" + fileName + ".txt", "r")
3) python HybridGA.py "name of instance" 
E.x: python HybridGA.py 100.10.1
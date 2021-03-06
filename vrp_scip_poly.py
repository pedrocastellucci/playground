from pyscipopt import Model, quicksum, Pricer, SCIP_RESULT, SCIP_PARAMSETTING

# Using networkx for drawing the solution:
import networkx as nx
import matplotlib.pyplot as plt

import numpy as np


class DataVRP:
    cap = None  # Capacity of the truck
    nodes = {}  # Position of the nodes
    depots = None  # We are assuming one depot
    demands = {}  # Demand for each node. Demand of the depot is zero
    costs = {}  # Costs from going from i to j
    
    def __init__(self, filename):
        
        with open(filename) as fd:
            lines = fd.readlines()

        i = 0
        size = None  # Number of nodes
        while i < len(lines):
            line = lines[i]

            if line.find("DIMENSION") > -1:
                size = self.sepIntValue(line)
            elif line.find("CAPACITY") > -1:
                self.cap = self.sepIntValue(line)            
            elif line.find("NODE_COORD_SECTION") > -1:
                i += 1
                line = lines[i]
                for _ in range(size):
                    n, x, y = [int(val.strip()) for val in line.split()]
                    i += 1  # Last step starts the DEMAND_SECTION
                    self.nodes[n] = (x, y)
                    line = lines[i]

            # Not elif because of NODE_COORD_SECTION process.
            if line.find("DEMAND_SECTION") > -1:
                i += 1
                line = lines[i]
                for _ in range(size):
                    n, dem = [int(val.strip()) for val in line.split()]
                    i += 1
                    self.demands[n] = dem
                    line = lines[i]

            if line.find("DEPOT_SECTION") > -1:
                i += 1
                depots = []
                
                depot = int(lines[i].strip())
                while depot >= 0:
                    depots.append(depot)
                    i += 1
                    depot = int(lines[i].strip())
                
                assert(len(depots) == 1)

                self.depot = depots[0]
            
            i += 1

        self.computeCostMatrix()


    def sepIntValue(self, line):
        value = line.split(":")[-1].strip()
        return int(value)

    def computeCostMatrix(self):
        for (p1, (x1, y1)) in self.nodes.items():
            for (p2, (x2, y2)) in self.nodes.items():
                self.costs[p1, p2] = ((x1 - x2)**2 + (y1 - y2)**2)**0.5


class VRPSolver(Pricer):

    data = None
    
    # List of client nodes:
    clientNodes = []

    # Model that holds the VRP problem (MTZ):
    model = None

    def __init__(self, vrpData):
        self.data = vrpData
        self.clientNodes = [n for n in self.data.nodes.keys() if n != self.data.depot]
        
    def solve(self, timeLimit):

        # Model for the vrp:
        subMIP = Model("VRP-MTZ")

        subMIP.setMinimize

        subMIP.setRealParam("limits/time", timeLimit)
        
        # Binary variables x_ij indicating whether the vehicle
        # traverses edge (i, j)
        x = {}
        for i in self.data.nodes:
            for j in self.data.nodes:
                if i != j:
                    x[i, j] = subMIP.addVar(vtype="B", obj=self.data.costs[i, j], name="x_%d_%d" % (i, j))

        # Non negative variables u_i indicating the demand served up to node i:
        u = {}
        for i in self.data.nodes:
            u[i] = subMIP.addVar(vtype="C", lb=0, ub=self.data.cap, obj=0.0, name="u_%d" % i)

        for j in self.clientNodes:
            subMIP.addCons(quicksum(x[i, j] for i in self.data.nodes if i != j) == 1)

        for h in self.clientNodes:
            subMIP.addCons(quicksum(x[i, h] for i in self.data.nodes if i != h) ==
                           quicksum(x[h, i] for i in self.data.nodes if i != h))

        for i in self.data.nodes:
            for j in self.clientNodes:
                if i != j:
                    subMIP.addCons(u[j] >= u[i] + self.data.demands[j]*x[i, j] - self.data.cap*(1 - x[i, j]))

        subMIP.optimize()

        self.model = subMIP

    def printSolution(self):
        edges = []
        print("Used edges:")
        for var in self.model.getVars():
            if var.name.find('x_') > -1 and self.model.getVal(var) > 0.01:
                print(var)
                # Getting the edge from variable name:
                
                i, j = var.name.split("_")[1:3]
                edges.append((int(i), int(j)))

        return edges

    def drawSolution(self):
        edges = self.printSolution()
        graph = nx.DiGraph()

        graph.add_edges_from(edges)

        nx.draw_networkx_nodes(graph, self.data.nodes)
        nx.draw_networkx_edges(graph, self.data.nodes)
        nx.draw_networkx_labels(graph, self.data.nodes)
        plt.show()


if __name__ == "__main__":
    data = DataVRP("./data/A-VRP/A-n32-k5.vrp")
#    data = DataVRP("toy15.vrp")

    solver = VRPSolver(data)
    solver.solve(40)  # 180 seconds (time limit)

    solver.printSolution()
    
    solver.drawSolution()


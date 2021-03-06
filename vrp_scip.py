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


class VRPpricer(Pricer):

    # Binary variables z_r indicating whether
    # pattern r is used:
    z = None

    # Data object with input data for the problem:
    data = None

    # Patterns currently in the problem:
    patterns = None

    # Function used to compute whether a
    # client is visited by a particular pattern.
    isClientVisited = None

    # Function used to compute the cost of a pattern:
    patCost = None

    # List of client nodes:
    clientNodes = []

    # Model that holds the sub-problem:
    subMIP = None

    # Maximum number of patterns to be created:
    maxPatterns = np.Inf
    
    def __init__(self, z, cons, data, patterns,
                 costs, isClientVisited, patCost, maxPatterns):
        
        self.z, self.cons, self.data, self.patterns = z, cons, data, patterns
        self.isClientVisited = isClientVisited
        self.patCost = patCost
        self.maxPatterns = maxPatterns

        for i in data.nodes:
            if i != data.depot:
                self.clientNodes.append(i)
        super()

    def pricerredcost(self):
        '''
        This is a method from the Pricer class.
        It is used for adding a new column to the problem.
        '''

        # Maximum number of patterns reached:
        print("Generated %d patterns" % len(self.patterns))
        if len(self.patterns) >= self.maxPatterns:
            print("Max patterns reached!")
            return {'result': SCIP_RESULT.SUCCESS}
        
        colRedCos, pattern = self.getColumnFromMIP(30)  # 30 seconds of time limit

        print(pattern, colRedCos)
        if colRedCos < -0.00001:

            newPattern = pattern
            obj = self.patCost(newPattern)
            curVar = len(self.z)
            newVar = self.model.addVar("New_" + str(curVar), vtype="C",
                                       lb=0.0, ub=1.0, obj=obj,
                                       pricedVar=True)

            for cs in self.cons:

                # Get client from constraint name:
                client = int(cs.name.split("_")[-1].strip())
                coeff = self.isClientVisited(client, newPattern)
                self.model.addConsCoeff(cs, newVar, coeff)

            self.patterns.append(newPattern)
            self.z[curVar] = newVar

        return {'result': SCIP_RESULT.SUCCESS}

    def pricerinit(self):
        '''
        A method of the Pricer class. It is used to convert
        the problem into its original form.
        '''
        for i, c in enumerate(self.cons):
            self.cons[i] = self.model.getTransformedCons(c)

    def getColumnFromMIP(self, timeLimit):

        def getPatternFromSolution(subMIP):
            edges = []
            for x in subMIP.getVars():
                if "x" in x.name:
                    if subMIP.getVal(x) > 0.99:
                        i, j = x.name.split("_")[1:]
                        edges.append((int(i), int(j)))
                    
            return edges

        # Storing the values of the dual solutions:
        dualSols = {}
        for c in self.cons:
            i = int(c.name.split("_")[-1].strip())
            dualSols[i] = self.model.getDualsolLinear(c)

        # Model for the sub-problem:
        subMIP = Model("VRP-Sub")
        subMIP.setPresolve(SCIP_PARAMSETTING.OFF)
        subMIP.setMinimize

        subMIP.setRealParam("limits/time", timeLimit)
        
        # Binary variables x_ij indicating whether the vehicle
        # traverses edge (i, j)
        x = {}
        for i in self.data.nodes:
            for j in self.data.nodes:
                if i != j:
                    x[i, j] = subMIP.addVar(vtype="B", obj=self.data.costs[i, j] - (dualSols[i] if i in self.clientNodes else 0), name="x_%d_%d" % (i, j))

        # Non negative variables u_i indicating the demand served up to node i:
        u = {}
        for i in self.data.nodes:
            u[i] = subMIP.addVar(vtype="C", lb=0, ub=self.data.cap, obj=0.0, name="u_%d" % i)

        for j in self.clientNodes:
            subMIP.addCons(quicksum(x[i, j] for i in self.data.nodes if i != j) <= 1)

        for h in self.clientNodes:
            subMIP.addCons(quicksum(x[i, h] for i in self.data.nodes if i != h) ==
                           quicksum(x[h, i] for i in self.data.nodes if i != h))

        for i in self.data.nodes:
            for j in self.clientNodes:
                if i != j:
                    subMIP.addCons(u[j] >= u[i] + self.data.demands[j]*x[i, j] - self.data.cap*(1 - x[i, j]))

        subMIP.addCons(quicksum(x[self.data.depot, j] for j in self.clientNodes) <= 1)

        subMIP.hideOutput()
        subMIP.optimize()

        mipSol = subMIP.getBestSol()
        obj = subMIP.getSolObjVal(mipSol)
        
        pattern = getPatternFromSolution(subMIP)
        
        return obj, pattern


class VRPsolver:

    data = None
    clientNodes = []

    # A pattern is a feasible route for visiting
    # some clients:
    patterns = None

    # The master model:
    master = None

    # The pricer object:
    pricer = None

    # Max patterns for column generation:
    maxPatterns = np.Inf

    # If we are solving the linear column generation this
    # must be False.
    # If it is True, we solve the problem of selecting
    # the best patterns to use -- without column generation.
    integer = False 
    
    def __init__(self, vrpData, maxPatterns):
        self.data = vrpData
        self.clientNodes = [n for n in self.data.nodes.keys() if n != self.data.depot]
        self.maxPatterns = maxPatterns
        
    def genInitialPatterns(self):
        ''' 
        Generating initial patterns.
        '''      
        patterns = []
        for n in self.clientNodes:
            patterns.append([(self.data.depot, n), (n, self.data.depot)])

        self.patterns = patterns

    def setInitialPatterns(self, patterns):
        self.patterns = patterns

    def addPatterns(self, patterns):
        for pat in patterns:
            self.patterns.append(pat)
        
    def patCost(self, pat):
        cost = 0.0

        for (i, j) in pat:
            cost += self.data.costs[i, j]
            
        return cost

    def isClientVisited(self, c, pat):
        # Check if client c if visited in pattern c:
        for (i, j) in pat:
            if i == c or j == c:
                return 1
        return 0

    def solve(self, integer=False):
        '''
        By default we solve a linear version of the column generation.
        If integer is True than we solve the problem of finding the best
        routes to be used without column generation.
        '''
        self.integer = integer
        if self.patterns is None:
            self.genInitialPatterns()

        # Creating master Model:
        master = Model("Master problem")

        # Creating pricer:
        master.setPresolve(SCIP_PARAMSETTING.OFF)

        # Populating master model.
        # Binary variables z_r indicating whether
        # pattern r is used in the solution:
        z = {}

        for i, _ in enumerate(self.patterns):
            z[i] = master.addVar(vtype="B" if integer else "C",
                                 lb=0.0, ub=1.0, name="z_%d" % i)

        # Set objective:
        master.setObjective(quicksum(self.patCost(p)*z[i] for i, p in enumerate(self.patterns)),
                            "minimize")
        
        print(master.getObjective())

        clientCons = [None]*len(self.clientNodes)
        
        for i, c in enumerate(self.clientNodes):
            cons = master.addCons(
            quicksum(self.isClientVisited(c, p)*z[i] for i, p in enumerate(self.patterns)) == 1,
                "Consumer_%d" % c,
                separate=False, modifiable=True)

            clientCons[i] = cons

        if not integer:
            pricer = VRPpricer(z, clientCons, self.data, self.patterns,
                               self.data.costs, self.isClientVisited,
                               self.patCost, self.maxPatterns)

            master.includePricer(pricer, "VRP pricer", "Identifying new routes")

            self.pricer = pricer
            
        self.master = master  # Save master model.
        
        master.optimize()

    def printSolution(self):

        if self.integer:
            zVars = self.master.getVars()
        else:
            zVars = self.pricer.z
        print(zVars)
        usedPatterns = []
        for i, z in enumerate(zVars):
            if self.master.getVal(zVars[i]) > 0.01:
                print(self.patterns[i], self.master.getVal(zVars[i]))
                usedPatterns.append(self.patterns[i])

        return usedPatterns

    def drawSolution(self):
        patterns = self.printSolution()
        graph = nx.DiGraph()

        for pat in patterns:
            graph.add_edges_from(pat)

        nx.draw_networkx_nodes(graph, self.data.nodes)
        nx.draw_networkx_edges(graph, self.data.nodes)
        nx.draw_networkx_labels(graph, self.data.nodes)
        plt.show()
        

if __name__ == "__main__":
    data = DataVRP("./data/A-VRP/A-n32-k5.vrp")
#    data = DataVRP("toy15.vrp")

    solver = VRPsolver(data, 180)
    solver.solve()

    usedPatterns = solver.printSolution()
    solver.drawSolution()

    solver.genInitialPatterns()
    solver.addPatterns(usedPatterns)

    solver.solve(integer=True)

    solver.drawSolution()

from pyscipopt import Model, quicksum, Pricer, SCIP_RESULT, SCIP_PARAMSETTING


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
                cap = self.sepIntValue(line)
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


ONE_PATTERN = True
class VRPpricer(Pricer):

    # Binary variables z_r indicating whether
    # pattern r is used:
    z = None

    # Data object with input data for the problem:
    data = None

    # Patterns currently in the problem:
    patterns = None

    # Dictionary costs[i, j] of moving from i to j:
    costs = None

    # Function used to compute whether a
    # client is visited by a particular pattern.
    isClientVisited = None

    # Function used to compute the cost of a pattern:
    patCost = None
    
    def __init__(self, z, cons, data, patterns,
                 costs, isClientVisited, patCost):
        
        self.z, self.cons, self.data, self.patterns = z, cons, data, patterns
        self.costs, self.isClientVisited = costs, isClientVisited
        self.patCost = patCost
        
        super()
    
    def pricerredcost(self):
        '''
        This is a method from the Pricer class.
        It is used for adding a new column to the problem.
        '''
        dualSols = []
        for i, c in enumerate(self.cons):
            dualSols.append(self.model.getDualsolLinear(c))

        print(dualSols, "DUALS")
        subMIP = Model("VRP-Sub")
        subMIP.setPresolve(SCIP_PARAMSETTING.OFF)

        global ONE_PATTERN
        if ONE_PATTERN:
            newPattern = [self.data.depot, 2, 3, 4]

            obj = self.patCost(newPattern)

            curVar = len(self.z)

            newVar = self.model.addVar("New_" + str(curVar), vtype="C", obj=obj, pricedVar=True)

            for cs in self.cons:

                # Get client from contraint name:
                client = int(cs.name.split("_")[-1].strip())

                coeff = self.isClientVisited(client, newPattern)

                self.model.addConsCoeff(cs, newVar, coeff)

            self.patterns.append(newPattern)
            self.z[curVar] = newVar

            ONE_PATTERN = False

        return {'result': SCIP_RESULT.SUCCESS}

    def pricerinit(self):
        '''
        A method of the Pricer class. It is used to convert
        the problem into its original form.
        '''
        for i, c in enumerate(self.cons):
            self.cons[i] = self.model.getTransformedCons(c)
    

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
    
    def __init__(self, vrpData):
        self.data = vrpData
        self.clientNodes = [n for n in self.data.nodes.keys() if n != self.data.depot]
        self.solve()
        
    def genInitialPatterns(self):
        ''' 
        Genereting initial patterns.
        '''
        
        patterns = []
        for n in self.clientNodes:
            patterns.append([self.data.depot, n])

        self.patterns = patterns

    def patCost(self, pat):
        cost = 0.0

        # Adding the depot to the end of the pattern:
        auxPatterns = pat + [self.data.depot]
        for (i, n) in enumerate(auxPatterns[:-1]):
            cost += self.data.costs[n, auxPatterns[i+1]]

        return cost

    def isClientVisited(self, c, pat):
        # Check if client c if visited in pattern c:
        if c in pat:
            return 1
        return 0

    def solve(self):
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
            z[i] = master.addVar(vtype="B", name="z_%d" % i)

        # Set objective:
        master.setObjective(quicksum(self.patCost(p)*z[i] for i, p in enumerate(self.patterns)),
                            "minimize")

        clientCons = [None]*len(self.clientNodes)
        
        for i, c in enumerate(self.clientNodes):
            cons = master.addCons(
            quicksum(self.isClientVisited(c, p)*z[i] for i, p in enumerate(self.patterns)) == 1,
                "Consumer_%d" % c,
                separate=False, modifiable=True)

            clientCons[i] = cons

        pricer = VRPpricer(z, clientCons, self.data, self.patterns,
            self.data.costs, self.isClientVisited, self.patCost)

        master.includePricer(pricer, "VRP pricer", "Identifying new routes")
       
        self.master = master  # Save master model.
        self.pricer = pricer
        
        master.optimize()

    def printSolution(self):
        zVars = self.pricer.z
        for i, z in enumerate(zVars):
            if self.master.getVal(zVars[i]) > 0.99:
                print(z, self.patterns[i])


if __name__ == "__main__":
    data = DataVRP("./data/A-VRP/A-n32-k5.vrp")
    solver = VRPsolver(data)

    solver.patCost([1, 2])
    solver.printSolution()
    

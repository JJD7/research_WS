import numpy as np
import math
import networkx as nx
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.ndimage.filters import gaussian_filter

class path_planner:

    def __init__(self):
        '''#####################################################################
            For every class instance, need to parse the data
            extracting node count, start node, and goal from the
            first three lines. The rest of the lines (4->inf) is edge info.
        #####################################################################'''

        self.n = 0
        self.start = 0
        self.goal = 0
        self.straightWeight = 1             #edge weights for N,S,E,W neighbors
        self.diagWeight = math.sqrt(2)*self.straightWeight      #edge weights for diagnal neighbors
        self.G= nx.empty_graph()
        self.color_map = []
        self.costMap = {}
        self.costMapViz = []
        self.O = {self.start:0.0}           #open list starts with starting node
        self.C = []                         #closed list is initially empty
        self.B = {self.start:self.start}    #backpointer dictionary. For now, start node points to itself
        self.V = {}                         #empty cost dictionary to be expanded later
        self.expansions = 0
        self.alpha = 20
        self.gamma = 0
        self.shortest_path = []

    def initCostMapGrid(self, costMap):
        cellSize = 16;
        frameSizeX = 3488;
        frameSizeY = 2560;
        cols = int(frameSizeX/cellSize);
        rows = int(frameSizeY/cellSize);
        self.n = cols*rows
        #self.costmapViz = np.reshape(costMap[:,2],(rows,cols))
        self.costMap = dict(zip(zip(costMap[:,0],costMap[:,1]),costMap[:,2]))
        self.G = nx.grid_2d_graph(rows,cols,periodic=False)
        for e in self.G.edges(): self.G[e[0]][e[1]]['weight']= self.straightWeight #set edge weights for N,S,E,W, neighbors
        for n in list(self.G.nodes()):
            r,c = n
            if 0 < r < rows-1 and 0 < c < cols-1:
                self.G.add_edge(n,(r-1,c-1),weight=self.diagWeight)
                self.G.add_edge(n,(r+1,c-1),weight=self.diagWeight)
                self.G.add_edge(n,(r+1,c+1),weight=self.diagWeight)
                self.G.add_edge(n,(r-1,c+1),weight=self.diagWeight)

    def set_Start_Goal(self, start, goal):
        self.start = start
        self.goal = goal
        self.O = {self.start:0.0}           #open list starts with starting node
        self.B = {self.start:self.start}    #backpointer dictionary. For now, start node points to itself
        self.shortest_path = [goal]
        for x in list(self.G.nodes()):         #Initialize cost to 0 only for start and inf for all others
            if x == start: self.V[x] = 0.0
            else: self.V[x] = float("inf")

    def huristic(self, node):
        # using euclidian distance huristic
        dx = self.goal[0]-node[0]
        dy = self.goal[1]-node[1]
        h = self.alpha*math.sqrt(dx**2+dy**2)
        return h

    def planner(self):   #actual planning Algorithm executes here
        V_new = 0.0
        while self.goal not in self.C:
            #get the node in the open list with the minimum cost to come plus huristic
            #add this node to closed list and remove from open list.
            xj = min(self.O, key=lambda i: self.V[i] + self.huristic(i))
            self.C.append(xj)
            del self.O[xj]
            self.expansions +=1         #incrementer to track performance

            if xj == self.goal: #this is the stop condition for the Algorithm
                print('goal reached in ', self.expansions,  'expansions')
                print('final cost is: ', self.V[self.goal])
                break

            #ignore neighboring nodes already in closed list
            #add neighbors to open list and compute the cost to come for each node
            for xi in list(self.G.neighbors(xj)):
                if xi in self.C: continue
                if xi not in self.O: self.O[xi]=0.0
                V_new = self.G[xj][xi]['weight'] + self.gamma*self.costMap[xi] + self.V[xj]

                #record the lowest cost so far
                #set backpoint to track shortest path so far
                if V_new < self.V[xi]:
                    self.V[xi] = V_new
                    self.B[xi] = xj
                    self.O[xi] = V_new

            if self.O is []:
                print("could not find path to goal")
                break

        #build shortest path from the backpointer dictionary working back from goal.
        pointer = self.goal

        #print('Shortest Path is:\n',goal)
        while pointer != self.start:
            #print(B[pointer])
            self.shortest_path.append(self.B[pointer])
            pointer = self.B[pointer]

def main():

    varMap = np.genfromtxt("cellVariance.txt", delimiter=',')
    cellSize = 16
    frameSizeX = 3488
    frameSizeY = 2560
    cols = int(frameSizeX/cellSize)
    rows = int(frameSizeY/cellSize)

    plan = path_planner()
    plan.initCostMapGrid(varMap)
    plan.set_Start_Goal((1,1),(120,200))
    plan.planner()

    blurred = gaussian_filter(np.reshape(varMap[:,2],(rows,cols)), sigma=0.5)

    fig = plt.figure()
    #ax = plt.axes(projection='3d')
    ax = fig.gca(projection='3d')
    path = np.asarray(plan.shortest_path)

    #ax.plot(path[:,1],path[:,0],0.2,color ='red',alpha=1,zorder=10,linewidth=3.0)
    #ax.scatter(plan.start[1],plan.start[0],0.2,color='green',zorder=11, s=20)
    #ax.scatter(plan.goal[1],plan.goal[0],0.2,color='black',zorder=11, s=20)
    surf = ax.plot_surface(np.reshape(varMap[:,1],(rows,cols)), np.reshape(varMap[:,0],(rows,cols)), blurred,cmap='winter',zorder=1)
    ax.view_init(azim=0, elev=90)
    fig.colorbar(surf)
    #hide grid
    ax.grid(False)
    # Hide axes ticks
    plt.axis('off')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.set_facecolor('black')
    plt.show()

if __name__== "__main__":
    main()

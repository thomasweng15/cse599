import numpy
import math
import networkx as nx
from DiscreteEnvironment import DiscreteEnvironment 

class SimpleEnvironment(object):
    
    def __init__(self, resolution, dimension):

        graphFile = 'graphs/' + `dimension` + 'D.graphml'
        self.graph = nx.read_graphml(graphFile)
        # You will use this graph to query the discrete world it embeds in the space

        self.lower_limits = numpy.zeros(dimension)
        self.upper_limits = numpy.ones(dimension)
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        
        obstacleFile = 'obstacles/' + `dimension` + 'D.txt'
        obstacles = []
        with open(obstacleFile) as file:
            for line in file:
                new_obstacle = [float(k) for k in line.split()]
                obstacles.append(new_obstacle)
        
        self.obstacles = numpy.array(obstacles)
        self.space_dim = dimension
        self.step_size = 0.01

    # Graph Methods
    def get_successors(self,v):
        '''
        @param v : vertex to expand
        returns the neighbours of vertex in graph
        '''
        # TODO: Here you will implement a function which 
        # takes in the vertex ID and returns all of its successors
        # in the graph.
        successors = []
        return successors

    def state_validity_checker(self,v):
        '''
        @param v: the state which is to be checked for collision
        return: 0 if free and 1 if in collision
        '''
        # TODO: Here you will implement a function which 
        # checks the collision status of a state represented 
        # by the the vertex ID given
        status = 0
        return status

    def edge_validity_checker(self,u,v):
        '''
        @param u,v: the states between which the edge is to be checked for collision
        returns 0 if free and 1 if in collision
        '''
        # TODO: Here you will implement a function which 
        # check the collision status of an edge between the states represented by the 
        # vertex IDs, using `step_size` as the resolution factor.
        status = 0
        return status

    def compute_distance(self,u,v):
        '''
        @param u,v: the states between which euclidean distance is to be computed
        returns euclidean distance between nodes
        '''
        # TODO: Here you will implement a function which computes the 
        # Euclidean distance between two states given their vertex IDs.
        dist = 0
        return dist

    def get_heuristic(self,v,t):
        '''
        @param v: node for which heuristic cost is to be determined
        @param t: target node
        returns the hueristic cost
        '''
        # TODO: Here you will implement a function to return the 
        # heuristic cost of a state given its vertex ID and goal ID
        heuristic = 0
        return heuristic

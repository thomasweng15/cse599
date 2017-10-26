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
        # Here you will implement a function which
        # takes in the vertex ID and returns all of its successors
        # in the graph.

        return list(self.graph.neighbors(v))

    def state_validity_checker(self,v):
        '''
        @param v: the state which is to be checked for collision
        return: 0 if free and 1 if in collision
        '''
        # Here you will implement a function which
        # checks the collision status of a state represented
        # by the the vertex ID given

        x, y = [float(a) for a in self.graph.nodes[v]['state'].split(' ')]
        for obs in self.obstacles:
            x_lower, y_lower, x_upper, y_upper = obs
            if (x_lower <= x and x <= x_upper) and (y_lower <= y and y <= y_upper):
                return 1

        return 0

    def edge_validity_checker(self,u,v):
        '''
        @param u,v: the states between which the edge is to be checked for collision
        returns 0 if free and 1 if in collision
        '''
        # TODO: Here you will implement a function which
        # check the collision status of an edge between the states represented by the
        # vertex IDs, using `step_size` as the resolution factor.

        # dist = self.compute_distance(u, v)

        status = 0
        return status

    def compute_distance(self,u,v):
        '''
        @param u,v: the states between which euclidean distance is to be computed
        returns euclidean distance between nodes
        '''
        # Here you will implement a function which computes the
        # Euclidean distance between two states given their vertex IDs.

        ux, uy = [float(a) for a in self.graph.nodes[u]['state'].split(' ')]
        vx, vy = [float(a) for a in self.graph.nodes[v]['state'].split(' ')]
        return math.sqrt((vx - ux)**2 + (vy - uy)**2)

    def get_heuristic(self,v,t):
        '''
        @param v: node for which heuristic cost is to be determined
        @param t: target node
        returns the hueristic cost
        '''
        # Here you will implement a function to return the
        # heuristic cost of a state given its vertex ID and goal ID

        return self.compute_distance(v, t)

if __name__ == '__main__':
    s = SimpleEnvironment(0.025, 2)

    # print s.get_successors('0')

    # for v in s.graph.nodes:
    #     if s.state_validity_checker(v) == 1:
    #         print v

    # for e in s.graph.edges:
    #     u, v = e
    #     if s.edge_validity_checker(u, v) == 1:
    #         print e

    # print s.compute_distance('0', '344')

    # print s.get_heuristic('0', '344')
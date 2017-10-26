import numpy
import math

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)

        print self.resolution, self.lower_limits, self.upper_limits, self.dimension, self.num_cells

    def configuration_to_nodeID(self, config):
        
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        coord = self.configuration_to_gridCoord(config)
        return self.gridCoord_to_nodeID(coord)

    def nodeID_to_configuration(self, nid):
        
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        coord = self.nodeID_to_gridCoord(nid)
        return self.gridCoord_to_configuration(coord)
        
    def configuration_to_gridCoord(self, config):
        
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        return [(int)(config[x] / self.resolution) for x in range(self.dimension)]

    def gridCoord_to_configuration(self, coord):
        
        # This function maps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        return [(coord[x] * self.resolution + self.resolution / 2) for x in range(self.dimension)]

    def gridCoord_to_nodeID(self,coord):
        
        # This function maps a grid coordinate to the associated
        # node id 
        return self.num_cells[1] * coord[0] + coord[1]

    def nodeID_to_gridCoord(self, node_id):
        
        # This function maps a node id to the associated
        # grid coordinate
        return [
            math.floor(node_id / self.num_cells[0]),
            node_id % self.num_cells[1]
        ]

if __name__ == '__main__':
    res = 0.0125
    dim = 2
    d = DiscreteEnvironment(res, numpy.zeros(dim), numpy.ones(dim))
    
    cfg = [0, 0.0125]
    node_id = d.configuration_to_nodeID(cfg)
    # print node_id
    
    config = d.nodeID_to_configuration(node_id)
    print config

    coord = d.configuration_to_gridCoord(cfg)
    # print coord

    cfg = d.gridCoord_to_configuration(coord)
    # print cfg

    node_id = d.gridCoord_to_nodeID(coord)
    # print node_id

    coord = d.nodeID_to_gridCoord(node_id)
    # print coord

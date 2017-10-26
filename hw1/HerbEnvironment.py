import numpy 
from DiscreteEnvironment import DiscreteEnvironment
import copy
import math

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        self.step_size = 0.05

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.gridCoord_to_configuration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def get_successors(self, node_id):

        # TODO: Here you will implement a function that returns all the 
        # neighbouring configurations' node IDs of the configuration
        # represented by the input `node_id`

        successors = []
        return successors

    def state_validity_checker(self, node_id):
        
        # TODO: Here you will implement a function which 
        # checks the collision status of a state represented 
        # by the the node ID given
        status = 0
        return status

    def edge_validity_checker(self, start_id, goal_id):

        # TODO: Here you will implement a function which 
        # check the collision status of an edge between the 
        # states represented by the node IDs given using  
        # using `step_size` as the resolution factor.
        status = 0
        return status
        
    def compute_distance(self, start_id, end_id):

        # TODO: Here you will implement a function which computes the 
        # distance between two states given their node IDs.
        dist = 0
        return dist

    def get_heuristic(self, start_id, goal_id):
        
        # TODO: Here you will implement a function to return the 
        # heuristic cost of a state given its node ID and goal ID
        heuristic = 0
        return heuristic

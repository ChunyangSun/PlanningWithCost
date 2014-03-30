import numpy
import IPython 
import copy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution, 0.2??
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


    def ConfigurationToNodeId(self, config):
        
        # 
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        # config = [0.26, 0.64] --> [1, 3]
        
        node_id = 0
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        
        return node_id
           

    def NodeIdToConfiguration(self, nid):
        
        # 
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        nid_copy = copy.copy(nid)
        coord = self.NodeIdToGridCoord(nid_copy)
        config = self.GridCoordToConfiguration(coord)

        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # 
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension

        for i in xrange(len(config)):
            coord[i] = numpy.floor((config[i] - self.lower_limits[i])/self.resolution) 

        return coord

    def GridCoordToConfiguration(self, coord):
        
        # 
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        # [1,3,2]--> [0.2, 0.6, 0.4]
        config = [0] * self.dimension

        for i in xrange(self.dimension):
            config[i] = self.lower_limits[i] + (coord[i] + 0.5)* self.resolution # at center of cell
        return config

    def GridCoordToNodeId(self,coord):
        
        # 
        # This function maps a grid coordinate to the associated
        # node id 
        # [1,3,2] --> 66 
        node_id = 0
        allPrevDimension = 1
        prev_node_id = coord[0]

        for i in xrange(self.dimension-1):
            allPrevDimension *= self.num_cells[i]
            node_id = coord[i+1] * allPrevDimension + prev_node_id
            prev_node_id = node_id
        
        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # 
        # This function maps a node id to the associated
        # grid coordinate
        # 66 --> [1,3,2]
        coord = [0] * self.dimension

        prev_dimensions = 1
        
        for i in xrange(self.dimension-1):
            prev_dimensions *= self.num_cells[i]

        for i in xrange(self.dimension-1, -1, -1):
            coord[i] = int(node_id) / int(prev_dimensions) 
            node_id -= coord[i] * prev_dimensions
            prev_dimensions /= self.num_cells[i-1] 

        return coord
print 
print "test = DiscreteEnvironment(0.2, [0,0,0], [1,1,1])"
test = DiscreteEnvironment(0.2, [0,0,0], [1,1,1])
print "testing ConfigurationToGridCoord([0.26, 0.64,0.6])... " 
print test.ConfigurationToGridCoord([0.26, 0.64,0.6])
print 
print "testing ConfigurationToNodeId([0.26,0.64,0.6])... " 
print test.ConfigurationToNodeId([0.26,0.64,0.6])
print 
print "testing NodeIdToConfiguration(66)... "
print test.NodeIdToConfiguration(66)
print "testing GridCoordToConfiguration([1,3,2])..."
print test.GridCoordToConfiguration([1,3,2])
print 
print "testing GridCoordToNodeId([1,3,2])..."
print test.GridCoordToNodeId([1,3,2])
print 

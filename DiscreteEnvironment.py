import numpy
import IPython 
import copy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):
        self.resolution = resolution       #default is 0.1, sent in run.py
        self.lower_limits = lower_limits   #bounds in resp. dimensions
        self.upper_limits = upper_limits
        self.dimension    = len(self.lower_limits)
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution) #number of cells in each dimension
        

    def ConfigurationToNodeId(self, config):
        coord   = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return    node_id
           
    def NodeIdToConfiguration(self, node_id):
        coord  = self.NodeIdToGridCoord(node_id)
        config = self.GridCoordToConfiguration(coord)
        return   config
        
    def ConfigurationToGridCoord(self, config):
        coord = [0] * self.dimension
        for i in xrange(len(config)):
            coord[i] = numpy.floor(((config[i] - self.lower_limits[i])/self.resolution)) #changed floor to around.
        return coord

    def GridCoordToConfiguration(self, coord):
        config = [0] * self.dimension
        for i in xrange(self.dimension):
            config[i] = self.lower_limits[i] + (coord[i] * self.resolution)           #rounding error, can ignore. not at centre, but node, so we can see path clearly
        return config

    def GridCoordToNodeId(self, coord):
        node_id = 0
        for i in xrange(len(coord)):
            node_id = node_id + coord[i]*numpy.prod(self.num_cells[:i])      #note np.prod(a[:i]) = a[0]*a[1]*a[2]*.....a[i-1] i      
        return node_id

    def NodeIdToGridCoord(self, node_id):
        dim   = self.dimension           #so that it is shorter         
        coord = [0] * dim
        temp  = node_id                  #can use node_id directly, but easy readability this way
        for i in xrange(self.dimension):
            coord[dim - (i+1)] = int(temp/numpy.prod(self.num_cells[:(dim-i-1)]))
            #print "prod obtained is ",  numpy.prod(self.num_cells[:(dim-i-1)])
            #print "quotiend we get is " , coord[dim - (i+1)] , "and the index is ", dim-(i+1)
            temp               = numpy.around(temp%numpy.prod(self.num_cells[:(dim-i-1)]))    #insurance against precision errors
            #print "remainder is ", temp

        return coord

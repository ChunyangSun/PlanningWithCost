import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.boundary_limits = [[-5., -5.], [5., 5.]] # for hRRT

        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        self.resolution   = resolution
        self.num_cells = self.discrete_env.num_cells
        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        self.table_p     = table.ComputeAABB().pos()
        self.table_e     = table.ComputeAABB().extents()
        self.herb_e      = self.robot.ComputeAABB().extents()



#################Note all of these will be using the input as node ids, and not grid or configurations####################
    def GetSuccessors(self, node_id):
	coord = self.discrete_env.NodeIdToGridCoord(node_id)
	cells = self.num_cells
	successors_node  = []
	for i in range(len(coord)):
		coord[i] = coord[i]+1
		if coord[i] >= 0 and coord[i] <= cells[i]-1:    #number of cells is x, but index ends at x-1
		    if not self.Collides(self.discrete_env.GridCoordToConfiguration(coord)):
			successors_node.append(self.discrete_env.GridCoordToNodeId(coord))
		coord[i] = coord[i]-2
		if coord[i] >= 0 and coord[i] <= cells[i]-1:    #number of cells is x, but index ends at x-1
		    if not self.Collides(self.discrete_env.GridCoordToConfiguration(coord)):
			successors_node.append(self.discrete_env.GridCoordToNodeId(coord))
	        coord[i] = coord[i]+1		    
	return successors_node


    def ComputeDistance(self, start_id, end_id):
	# Euclidian distance
        coord1 = self.discrete_env.NodeIdToGridCoord(start_id)
	coord2 = self.discrete_env.NodeIdToGridCoord(end_id)
#	num_dof = len(coord1)
#	dist = 0
#	for i in range(num_dof):
#	    dist = dist + abs(coord1[i] - coord2[i]) 
#The actual path will be the euclidian distance,(this wont work in in case we take 8 neighbours)
       # print "The coord1 is " , coord1, " the coord2 is" , coord2
        return numpy.linalg.norm(numpy.array(coord1)-numpy.array(coord2))


    def ComputeHeuristicCost(self, start_id, goal_id):
        coord1 = self.discrete_env.NodeIdToGridCoord(start_id)
	coord2 = self.discrete_env.NodeIdToGridCoord(goal_id)
        return numpy.linalg.norm(numpy.array(coord1)-numpy.array(coord2))

    def Collides(self, config):                        #remember to pass the configuration. and note this is only in simple case
	T = self.robot.GetTransform()
	Tnew = T
	Tnew[[0,1],3] = config
	env = self.robot.GetEnv()
	self.robot.SetTransform(Tnew)
	return env.CheckCollision(self.robot, env.GetKinBody('conference_table'))

    def send_close_list(self,e_bot,e_obj,p_obj):
        lin_x=numpy.linspace(p_obj[0]+e_obj[0]+e_bot[0],p_obj[0]-e_obj[0]-e_bot[0],int(4*(e_bot[0]+e_obj[0])/self.resolution))
        lin_y=numpy.linspace(p_obj[1]+e_obj[1]+e_bot[1],p_obj[1]-e_obj[1]-e_bot[1],int(4*(e_bot[1]+e_obj[1])/self.resolution))
        bad_list = []
	for i in range(len(lin_x)):
	        bad_list.append([self.discrete_env.ConfigurationToNodeId([lin_x[i], p_obj[1]+e_obj[1]+e_bot[1]]),-20,-20,-20,-20])
		bad_list.append([self.discrete_env.ConfigurationToNodeId([lin_x[i], p_obj[1]-e_obj[1]-e_bot[1]]),-20,-20,-20,-20])
        for i in range(len(lin_y)):
		bad_list.append([self.discrete_env.ConfigurationToNodeId([p_obj[0]-e_obj[0]-e_bot[0],lin_y[i]]),-20,-20,-20,-20])
		bad_list.append([self.discrete_env.ConfigurationToNodeId([p_obj[0]+e_obj[0]+e_bot[0],lin_y[i]]),-20,-20,-20,-20])
        return bad_list


############################# for hRRT ################################
    def GenerateRandomConfiguration(self):
        config = numpy.random.random(2)
        limits = numpy.array(self.boundary_limits)
        config = (config * (limits[1] - limits[0])) + limits[0]
        return config if self.isValid(config) else self.GenerateRandomConfiguration()

    def ComputeDistanceRRT(self, start_config, end_config):
        dist = numpy.sqrt((start_config[0] - end_config[0])**2 + (start_config[1] - end_config[1])**2)
        return dist

    def Extend(self, start_config, end_config):
        configs = numpy.array([start_config, end_config])
        last = start_config
        for tau in numpy.arange(0.0, 1.01, 0.01):
            newconfig = numpy.average(configs, axis=0, weights=[1.0-tau, tau])
            if(self.isValid(newconfig)):
                last = newconfig
            else:
                break
        # return the last collision free point 
        return last
       
        #returns true if a point is not in collision
    def isValid(self, point):
        T = numpy.identity(4)
        T[0,3] = point[0]
        T[1,3] = point[1]
        self.robot.SetTransform(T)
        return not self.robot.GetEnv().CheckCollision(self.robot)

    def ComputePathLength(self, path):      
        i = 1
        length = 0.0
        while i < len(path):
            length += self.ComputeDistance(path[i-1], path[i])
            i += 1
        return length

    def ShortenPath(self, path, timeout=5.0):
        start = end = time.time()
        lastPath = []

        # stop when there's no more shortening to be done 
        while not numpy.array_equal(path, lastPath): 
            lastPath = copy.deepcopy(path)

            # keep checking alternate adjacent nodes  
            idx = 1
            while idx < (len(path) - 1):
                n1 = path[idx - 1] 
                n2 = path[idx + 1]

                if numpy.array_equal(self.Extend(n1, n2), n2):
                    #delete the selected node
                    path = numpy.delete(path, idx, axis = 0)
                idx += 1
            # break out of while loop when time out 
            if (time.time() - start) > timeout: break
        return path

################### for Visualization ##########################
    def InitializePlot(self, goal_config):  #default
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):   #default
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

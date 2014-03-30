import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):
	coord = NodeIdToGridCoord(node_id)
	num_dof = len(coord)
	successors = []
	for i in range(num_dof):
	    new_coord = coord
	    if coord[i] > 0:
		new_coord[i] = coord[i] - 1
		new_config = GridCoordToConfiguration(new_coord)
		if Collides(new_config) == False:
		    new_node = GridCoordToNodeId(new_coord)
		    successors.append(new_node)
	    if coord[i] < self.numCells[i]:
		new_coord[i] = coord[i] + 1
		new_config = GridCoordToConfiguration(new_coord)
		if Collides(new_config) == False:
		    new_node = GridCoordToNodeId(new_coord)
		    successors.append(new_node)
        return successors

    def ComputeDistance(self, start_id, end_id):
	# manhattan distance
        coord1 = NodeIdToGridCoord(start_id)
	coord2 = NodeIdToGridCoord(end_id)
	num_dof = len(coord1)
	dist = 0
	for i in range(num_dof):
	    dist = dist + abs(coord1[i] - coord2[i])
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        cost = ComputeDistance(start_id, end_id)
        return cost

    def Collides(self, config)
	T - self.robot.GetTransform()
	Tnew = T
	Tnew[[0,1],3] = config
	env = self.robot.GetEnv()
	self.robot.SetTransform(Tnew)
	return env.CheckCollision(self.robot, env.GetKinBody('conference_table'))

    def InitializePlot(self, goal_config):
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
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

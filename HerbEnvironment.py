import numpy
import copy
import time
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.boundary_limits = [[-5., -5.], [5., 5.]]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
	self.num_cells    = self.discrete_env.num_cells
	self.manip        = self.robot.GetActiveManipulator()
        self.resolution    = resolution
        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
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
        self.table_p     = table.ComputeAABB().pos()       #we wont be using these here. sort of dummy value
        self.table_e     = table.ComputeAABB().extents()
        self.herb_e      = self.robot.ComputeAABB().extents()
    
    def GetSuccessors(self, node_id):
        successors_node = []

	coord = self.discrete_env.NodeIdToGridCoord(node_id)
	cells = self.num_cells
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
        


    def GetSuccessorsCord(self, coord):
	cells = self.num_cells
	successors_node  = []
	for i in range(len(coord)):
		coord[i] = coord[i]+1
		if coord[i] >= 0 and coord[i] <= cells[i]-1:    #number of cells is x, but index ends at x-1
		    if not self.Collides(self.discrete_env.GridCoordToConfiguration(coord)):
                        temp = copy.copy(coord)
			successors_node.append(temp)
		coord[i] = coord[i]-2
		if coord[i] >= 0 and coord[i] <= cells[i]-1:    #number of cells is x, but index ends at x-1
		    if not self.Collides(self.discrete_env.GridCoordToConfiguration(coord)):
			temp = copy.copy(coord)
			successors_node.append(temp)
	        coord[i] = coord[i]+1		    
#	import IPython
#	IPython.embed()

	return successors_node



    def ComputeDistance(self, start_id, end_id):
        coord1 = self.discrete_env.NodeIdToGridCoord(start_id)  #note that we call computedistance only for neighbours, 
	coord2 = self.discrete_env.NodeIdToGridCoord(end_id)    #so the computedistance will give out only the manhattandistance type result 
        return numpy.linalg.norm(numpy.array(coord1)-numpy.array(coord2))


    def ComputeDistanceCord(self, coord1, coord2):
      #  coord1 = self.discrete_env.NodeIdToGridCoord(start_id)  #note that we call computedistance only for neighbours, 
#	coord2 = self.discrete_env.NodeIdToGridCoord(end_id)    #so the computedistance will give out only the manhattandistance type result 
        return numpy.linalg.norm(numpy.array(coord1)-numpy.array(coord2))



    def ComputeHeuristicCost(self, start_id, goal_id):          #Testing regular heuristic first
        coord1 = self.discrete_env.NodeIdToGridCoord(start_id)
	coord2 = self.discrete_env.NodeIdToGridCoord(goal_id)
        config1 = self.discrete_env.GridCoordToConfiguration(coord1)
        config2 = self.discrete_env.GridCoordToConfiguration(coord2)
        return numpy.linalg.norm(numpy.array(coord1)-numpy.array(coord2))  #Euclidian distance heuristic in 7dof
	with self.robot.GetEnv():
                self.robot.SetActiveDOFValues(config1)
                T_start = self.manip.GetEndEffectorTransform()

                self.robot.SetActiveDOFValues(config2)
                T_end   = self.manip.GetEndEffectorTransform()


#        return numpy.sum(numpy.fabs(numpy.array(coord1) -numpy.array(coord2)))                       #Manhattan distance in 7dof
        return (numpy.linalg.norm(T_start - T_end)/self.resolution)#+10*numpy.sum(numpy.fabs(numpy.array(coord1) -numpy.array(coord2)))                       #Manhattan distance in 7dof                     #Manhattan distance in 7do
     


    def ComputeHeuristicCostCord(self, coord1, coord2):          #Testing regular heuristic first
#        coord1 = self.discrete_env.NodeIdToGridCoord(start_id)
#	coord2 = self.discrete_env.NodeIdToGridCoord(goal_id)
        config1 = self.discrete_env.GridCoordToConfiguration(coord1)
        config2 = self.discrete_env.GridCoordToConfiguration(coord2)
 #       return numpy.linalg.norm(numpy.array(coord1)-numpy.array(coord2))  #Euclidian distance heuristic in 7dof
#	with self.robot.GetEnv():
 #               self.robot.SetActiveDOFValues(config1)
  #              T_start = self.manip.GetEndEffectorTransform()

   #             self.robot.SetActiveDOFValues(config2)
    #            T_end   = self.manip.GetEndEffectorTransform()


        return numpy.sum(numpy.fabs(numpy.array(coord1) -numpy.array(coord2)))                       #Manhattan distance in 7dof
 #       return (numpy.linalg.norm(T_start - T_end)/self.resolution)#+10*numpy.sum(numpy.fabs(numpy.array(coord1) -numpy.array(coord2)))                       #Manhattan distance in 7dof                     #Manhattan distance in 7do

    def ComputePathLength(self, plan):      
        dist = 0 
        for i in range(0,len(plan)-1):
            x = self.ComputeDistanceCord(plan[i],plan[i+1])
            dist = dist + x       
        return dist


    def Collides(self, config):                        #remember to pass the configuration. and note this is only in simple case
	with self.robot.GetEnv():
	    self.robot.SetActiveDOFValues(config)
	    flag1 = self.robot.GetEnv().CheckCollision(self.robot, self.robot.GetEnv().GetKinBody('conference_table'))
	  #  flag2 = self.robot.CheckSelfCollision()
	return not( (not flag1)) #and (not flag2))

    def send_close_list(self,e_bot,e_obj,p_obj): #written so that we have same Astarplanner for both herb and simple env
        return []


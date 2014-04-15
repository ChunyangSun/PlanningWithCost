
class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_node, goal_node):
        
        if self.visualize:
	    self.planning_env.InitializePlot(self.planning_env.discrete_env.NodeIdToConfiguration(goal_node))

        plan = []
        open_list  = []
        close_list = []#self.planning_env.send_close_list(self.planning_env.herb_e,self.planning_env.table_e,self.planning_env.table_p)
########################## Note, write send_close_list function in herbenvironment, that passes an empty list.#########
        No_path      =  0                                                                  #flag for no path	
        open_list.append([start_node,start_node])
        neighbour    = self.planning_env.GetSuccessors(start_node)                         #initializations    
        current_node = open_list.pop(0)                                                     #remove from the open list the node to expand 
        close_list.append(current_node)                                                         

        while(current_node[0] != goal_node  and No_path == 0):    
		for i in range(len(neighbour)):
                    [node,index] = present_in_list(neighbour[i],open_list)
                    if index != []:
			pass    #once we expand a node, we dont do anything
		    else:
			[node2,index2] = present_in_list(neighbour[i],close_list) #also check if this has been expanded and put in closed_list
			if index2 == []:
		            open_list.append([neighbour[i],current_node[0]]) #append it only if it is not present in closed list.
	                
        	if len(open_list) == 0:
            	    No_path = 1     
            	    print "No path exist"
		else:	        
	    	    current_node  = open_list.pop(0)      #remove from the open list the node to expand 
	            close_list.append(current_node)         #add the node that has been expanded to the closed list. will be [id,p_id,fn,gn,tc]
	    	    try:                                        # throws exception when list is empty. hence try catch
			neighbour = self.planning_env.GetSuccessors(current_node[0])           
		    except ValueError:                          # returns a null array when the list is empty.
			neighbour = []
		    
	if No_path == 1:
           # print "we added the last node here"
            close_list.append([goal_node,current_node[0]]) # to add the last point
        
       # print "the closed list is ",close_list ," and length of list is ", len(close_list)
        path_return_node  = [] 
        path_return_coord = []
        parent_node = goal_node
	while parent_node != start_node:
	    index_node   = list(zip(*close_list)[0]).index(parent_node)
            path_return_node.append(close_list[index_node][0])           #note that once again, the coordinates are the node_ids  
    	    path_return_coord.append(self.planning_env.discrete_env.NodeIdToConfiguration(close_list[index_node][0]))
            parent_node  = close_list[index_node][1]

        print "Close list size: " + str(len(close_list))  
       # print "The plan is parent return coord", path_return_coord
        plan = path_return_coord[::-1]
        if self.visualize:
            for i in range(len(plan)-1):
	        self.planning_env.PlotEdge(plan[i],plan[i+1])

       # print "the plan is ",plan
        return plan

def present_in_list(point,open_list):
	try:
	    index = list(zip(*open_list)[0]).index(point)
	    return [open_list[index],index]
        except (IndexError,ValueError) as e:
            return [[],[]]

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        

import numpy as np
import collections as col
class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    #each node will have the following information (x_curr,y_curr,x_par,y_par,cost_from_start,heuristic,total_cost) 
    #x_par is the parent coordinates
    
    def Plan(self, start_node, goal_node):

	if self.planning_env.robot.GetName() == 'pr2':
	    heuristic_factor = 1
	else:
	    heuristic_factor = 2
 

	start_cord = self.planning_env.discrete_env.NodeIdToGridCoord(start_node)
	goal_cord = self.planning_env.discrete_env.NodeIdToGridCoord(goal_node)

        if self.visualize:
	    self.planning_env.InitializePlot(goal_cord)
     
        plan       = []
	open_list  = []
        close_list = self.planning_env.send_close_list(self.planning_env.herb_e,self.planning_env.table_e,self.planning_env.table_p)
        No_path      =  0                                                                  #flag for no path
	path_cost    =  0
     
        dist_goal    = self.planning_env.ComputeHeuristicCostCord(start_cord, goal_cord)      
        open_list.append([start_cord,start_cord,path_cost,dist_goal,dist_goal])
        neighbour    = self.planning_env.GetSuccessorsCord(start_cord)                         #initializations    
        popped       = open_list.pop()                                                     #remove from the open list the node to expand 
        close_list.append(popped)                                                         
        current_cord = popped

	#print "current node is ",current_cord,"with first element",current_cord[0]
        while(col.Counter(np.subtract(current_cord[0],np.array(goal_cord))) != col.Counter([0]*len(current_cord[0]))  and No_path == 0):    
	    for i in range(len(neighbour)):
                neigh_path_cost = popped[2]+self.planning_env.ComputeDistanceCord(popped[0],neighbour[i])
		neigh_heu       = heuristic_factor*self.planning_env.ComputeHeuristicCostCord(neighbour[i],goal_cord)
                [node,index] = present_in_list(neighbour[i],open_list)      #check neighbour is in the open list, and return the node and index
	        if index != []:
                    if  open_list[index][4] >= neigh_path_cost + neigh_heu: # part where we swap the minimum values
                        open_list[index][1] = popped[0]                     # update the parent
                        open_list[index][2] = neigh_path_cost               # update the path cost
 		        open_list[index][3] = neigh_heu                     # update heuristic
		        open_list[index][4] = neigh_path_cost + neigh_heu   # update total cost
	#		print "we are changing the cost"
   	        else:
                    [no_2,ind_2] = present_in_list(neighbour[i],close_list) 
		    if ind_2 ==[]:
	                open_list.append([neighbour[i],popped[0],neigh_path_cost,neigh_heu,neigh_path_cost+neigh_heu])       
#                        print "and we ahve appended ",open_list[-1]
#	    print "The length of open list is ",len(open_list)
  	    [index_node, current_cord] = find_min(open_list)  #get the point and index of min cos/
	    #print "the node that we picked up is ",current_cord," and the goal coord is ", goal_cord
            #if (current_cord[0][0] == goal_cord[0]) and (current_cord[0][1] == goal_cord[1]):
		#print "current cord is " , current_cord, " goal cord is ", goal_cord
#		import IPython
#		IPython.embed()
	 #   import IPython 
	 #   IPython.embed()
            if current_cord == []:                         #If open list is empty  
                No_path = 1                                #change flag, so as to break while loop
		print "No path exists"
            else:
                popped     = open_list.pop(index_node)      #remove from the open list the node to expand 
                close_list.append(popped)                   #add the node that has been expanded to the closed list. will be [id,p_id,fn,gn,tc]
	        try:                                        # throws exception when list is empty. hence try catch
                  #  print "current_cord is ", current_node 
                    neighbour = self.planning_env.GetSuccessorsCord(popped[0])           
		   # print "The length of neighbours is ",len(neighbour)
                except ValueError:                          # returns a null array when the list is empty.
                    neighbour = []

      
        ################end of while loop
        if No_path == 1:
            close_list.append([goal_cord,current_cord[0],current_cord[4],0,current_cord[4]]) # to add the last point

	#print "close_list is ",close_list
        ################retrace from goal to start
	path_return_config  = []
        path_return_cord    = []
        parent_cord = goal_cord
	while col.Counter(np.subtract(parent_cord,start_cord)) != col.Counter([0]*len(start_cord)): 
            [no_2,ind_2] = present_in_list(parent_cord,close_list) 
            path_return_cord.append(no_2[0])           #note that once again, the coordinates are the node_ids  
            parent_cord  = no_2[1]
    	    path_return_config.append(self.planning_env.discrete_env.GridCoordToConfiguration(no_2[0])) #######ToDO HERE
#	    import IPython 
#	    IPython.embed()

        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        #print "path return coord is ",path_return_cord

	#print "Close list size: " + str(len(close_list))
       # print "the coord of path are", path_return_coord 
        plan = path_return_config[::-1]
        if self.visualize:
            for i in range(len(plan)-1):
	        self.planning_env.PlotEdge(plan[i],plan[i+1])
        #print "plan is" ,plan
        return plan


################## This funtion will stay here itself ##############                    
def find_min(list_given): #function to find out the min value and the index in the openlist
        flag = 1
        try:
            total_min = min(list_given, key=lambda x: x[4])
        except ValueError:
            flag = 0
        try:                           # throws exception when list is empty. hence try catch
            index_min = list_given.index(min(list_given,key=lambda x: x[4]))
        except ValueError:             # returns a null array when the list is empty.
            flag = 0
        if flag == 0:
            return [[],[]]
        else:
            return [index_min,total_min]  

#def present_in_list(point,open_list):
#        try:
#            temp_list = list(zip(*open_list)[0])
#            temp      = []
#           # import IPython
#           # IPython.embed()
#            for i in range(len(temp_list)):
#                if col.Counter(np.subtract(np.array(temp_list[i]),np.array(point))) == col.Counter([0]*len(point)):
#               # if set(temp_list[i]) == set(point):
#                    temp = [open_list[i],i]
#            if temp != []:
#                return temp
#            else:
#                return [[],[]]
#        except (IndexError,ValueError) as e:
#            return [[],[]]

def present_in_list(point,open_list):
        try:
            temp      = []
	    FLAG = 0
	    for i in range(len(open_list)):
        	temp_1 = np.array(open_list[i][0])-np.array(point)
        	if np.any(temp_1) == False:
                    temp = [open_list[i],i]
                    FLAG = 1
	            break
	    if FLAG == 1:
                return temp
            else:
                return [[],[]]
        except (IndexError,ValueError) as e:
            return [[],[]]

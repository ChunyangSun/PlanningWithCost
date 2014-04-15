def compute_distance(first_node, second_node):
        return np.linalg.norm(first_nord - second_node) # euclidian distance


def compute_heuristic(first_node, second_node):
        return np.linalg.norm(first_nord - second_node) # heuristic is also the euclidian distance

#def expand(node_to_expand, hn_value, goal_node, closed_list , limits_grid, no_neighbours = 4)
#        if no_neightbours == 8 #we will have to change the things for the nuber of neigbours
                               #GetSuccessor funtion will return the good values here

def find_min(list_given): #function to find out the min value and the index in the openlist
    # using the folowing code for numpy arrays might be useful
     #    np.argmax(np.transpose(temp)[4])
        try:                           # throws exception when list is empty. hence try catch
            total_min = min(list_given, key=lambda x: x[4])
            index_min = list_given.index(min(list_given,key=lambda x: x[4]))
            return [index_min, total_min]
        except ValueError:             # returns a null array when the list is empty.
             return []

def Plan(start_config, goal_config):

    plan = []
    #Initializations
    current_node =  start_config  #current node will be the node we are tracking
    index        =  0
    path_cost    =  0   
    dist_goal    =  compute_heuristic(current_node, goal_config)
    open_list.append([current_node,current_node,path_cost,dist_goal, dist_goal ])
    No_path      =  1  #flag that we will be using

    while(current_node != goal_config and No_path == 1):

        neighbours = expand(current_node)
        open_list.delete(current_node, index)
        close_list.append(current_node)

    for i in range(neighbours):
        if present_in_open(neighbours[i][1] , open_list):
            open_list = min                                      # part where we swap the minimum values
        else:
            open_list.append(neighbour[i])        # append the appropriate neighbour
        
    current_node,index_node = find_min(open_list)
    if current_node = []:
        No_path = 0
        
    # TODO: Here you will implement the AStar planner
    #  The return path should be a numpy array
    #  of dimension k x n where k is the number of waypoints
    #  and n is the dimension of the robots configuration space

    plan.append(start_config)
    plan.append(goal_config)

return plan
                                                                                                                         70,0-1        Bot


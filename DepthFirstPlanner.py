from RRTTree import RRTTree

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        
        plan = []
        
        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan.append(start_config)
        if dfs_rec(start_config) == None: return [] # TODO: or return None
        else: 
            print "Tree size: " + str(len(self.tree.vertices))
          
            currVertex = eid
            while (currVertex != tree.GetRootId()):
                currVertex = tree.edges[currVertex] 
                plan.insert(0, tree.vertices[currVertex]) 
            plan.append(goal_config)

            return plan

    def dfs_rec(parent):
        # base case, no children 
        if(self.planning_env.GetSuccessors(parent) == None):
            return None
        elif(parent == goal_config):
            return 
        else:
            Children = self.planning_env.GetSuccessors(parent) 
            for child in children:
                # avoid revisit and check collision
                if child not in recordMarked and (not self.s.Collides(child)):
                    recordMarked.append(child)
                    dfs_rec(child)
                    sid = self.recordMarked.index(parent)
                    eid = self.tree.AddVertex(child)
                    self.tree.AddEdge(self, sid, eid)
                    # visualize the updated tree
                    self.planning_env.PlotEdge(self, parent, child)



                   



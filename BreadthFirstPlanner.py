import numpy
from SimpleEnvironment import SimpleEnvironment

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        plan = []
        recordMarked = []
        self.tree = RRTTree(planning_env, start_config)
        sid = 0
        
        plan.append(start_config)   
        queue = [start_config]
        while len(queue) != 0:
            # pop the first element in queue FIFO
            parent = queue.pop(0)
            # see if that's the goal
            if parent == goal_config:
                break
            else:                 
                 Children = self.s.GetSuccessors(parent) 
                 for child in children:
                    # avoid revisit and check collision
                    if child not in recordMarked and (not self.s.Collides(child)):
                        recordMarked.append(child)
                        queue.append(child)
                        sid = self.recordMarked.index(parent)
                        eid = self.tree.AddVertex(child)
                        self.tree.AddEdge(self, sid, eid)
                        #visualize the updated tree
                        self.planning_env.PlotEdge(self, parent, child)
                   

        print "Tree size: " + str(len(self.tree.vertices))
        plan = []
        currVertex = eid
        while (currVertex != tree.GetRootId()):
            currVertex = tree.edges[currVertex] 
            plan.insert(0, tree.vertices[currVertex]) 
        plan.append(goal_config)

        return plan


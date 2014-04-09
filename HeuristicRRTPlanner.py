import numpy
from RRTTree import RRTTree
from SimpleEnvironment import SimpleEnvironment
from SimpleRobot import SimpleRobot
import random
import IPython

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_id, goal_id, epsilon = 0.001):
        self.start_config = self.planning_env.discrete_env.NodeIdToConfiguration(start_id) 
        self.goal_config = self.planning_env.discrete_env.NodeIdToConfiguration(goal_id)
        self.tree = RRTTree(self.planning_env, self.start_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(self.goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        q_new = self.start_config
        q_select = None 
        count = 0
        dist = 9999
        vid_end = 0
        prob = 0.5 
        self.minCost = self.getHCost(self.start_config, self.goal_config)
        self.maxCost = self.minCost
        # if q_new is very near to the goal, break out of the loop 
        # don't worry about goal not included in vertices or edges: start and goal are in path 
        while (dist > epsilon) : 
            while (q_select == None): # select q_rand untill you find a q_new that is low cost 
            # Pick a sample q_rand in space C, sample the goal every 5 cycles 
                q_rand  = self.planning_env.GenerateRandomConfiguration() if (count % 5) else self.goal_config
                count += 1
                q_near, q_select, vid_near, q_select_cost = self.selectNode(q_rand)
            (q_near, q_new, vid_start, q_new_cost) = (q_near, q_select, vid_near, q_select_cost)
            q_select = None # reset q_select
            
            # IPython.embed()
            
            # add q_new to the tree, #TODO: possible improvement, detect/avoid duplicates
            vid_end = self.tree.AddVertex(q_new)
            self.tree.AddEdge(vid_start, vid_end)
            self.maxCost = max(self.maxCost, q_new_cost)
            # update the distance from the goal, improvement: calculated dist twice
            dist = self.getHCost(q_new, self.goal_config)
            print "maxCost: ", self.maxCost
            print "vid_end: ", vid_end 

            #visualize the updated tree
            if self.visualize:
                self.planning_env.PlotEdge(q_near, q_new) 

        plan = []
        currVertex = vid_end
        while (currVertex != self.tree.GetRootId()):
            currVertex = self.tree.edges[currVertex] 
            plan.insert(0, self.tree.vertices[currVertex]) 
        plan.append(self.goal_config)

        return plan

    def selectNode(self, q_rand):
        thr = 0.5
        # find q_rand's nearest milestone q in the tree:    
        [vid_near, q_near] = self.tree.GetNearestVertexRRT(q_rand)  # 2D array  
        
        q_select = self.planning_env.Extend(q_near, q_rand)
        q_select_cost = self.getGCost(vid_near) + self.getHCost(q_near, q_select) + self.getHCost(q_select, self.goal_config)
        nodeQuality = self.getNodeQuality(q_select, q_select_cost)
        print "q_near: ", q_near, vid_near
        print "q_select: ", q_select, q_select_cost
        print "q_select_cost", q_select_cost
        print "nodeQuality: ", nodeQuality
        if nodeQuality > thr: 
            return q_near, q_select, vid_near, q_select_cost
        else: return None, None, None, None
        # IPython.embed()

    def getNodeQuality(self, q_select, q_select_cost):
        if self.maxCost == self.minCost: 
            return 1 
        return 1 - (q_select_cost - self.minCost) / (self.maxCost - self.minCost)

    def getHCost(self, config1, config2):
        return numpy.sqrt((config2[0] - config1[0])**2 + (config2[1] - config1[1])**2)
        
    def getGCost(self, currId):
        gCost = 0

        while (currId != self.tree.GetRootId()):
            lastId = currId
            lastConfig = self.tree.vertices[lastId]
            currId = self.tree.edges[currId] 
            currConfig = self.tree.vertices[currId]
            gCost += self.getHCost(currConfig, lastConfig)
        
        return gCost 








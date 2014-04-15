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
        
    def Plan(self, start_id, goal_id, epsilon = 0.01):
        self.start_config = numpy.array(self.planning_env.discrete_env.NodeIdToConfiguration(start_id))
        self.goal_config = numpy.array(self.planning_env.discrete_env.NodeIdToConfiguration(goal_id))
        self.tree = RRTTree(self.planning_env, self.start_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(self.goal_config)

        q_new = self.start_config
        q_select = None 
        count = 0
        dist = 9999
        vid_end = 0
        prob = 0.6 
        thr = 0.7
        self.minCost = self.getHCost(self.start_config, self.goal_config)
 
        # if q_new is very near to the goal, break out of the loop 
        # don't worry about goal not included in vertices or edges: start and goal are in path 
        q_rand  = self.planning_env.GenerateRandomConfiguration() 
        vid_near, q_near = self.tree.GetNearestVertexRRT(q_rand)
        q_new = self.planning_env.Extend(q_near, q_rand)
        self.maxCost = self.getGCost(vid_near) + self.getHCost(q_near, q_new) + self.getHCost(q_new, self.goal_config) 

        while (dist > epsilon): 
            while (True): # select q_rand untill you find a q_new that is low cost 
                # Pick a sample q_rand in space C, sample the goal every 5 cycles 
                q_rand  = self.planning_env.GenerateRandomConfiguration() if (count % 5) else self.goal_config
                count += 1
                
                # find nearest neighbor in the tree 
                [vid_near, q_near] = self.tree.GetNearestVertexRRT(q_rand)
                
                q_select = self.planning_env.Extend(q_near, q_rand)
                q_select_cost = self.getGCost(vid_near) + self.getHCost(q_near, q_select) + self.getHCost(q_select, self.goal_config) 
                q_select_quality = self.getNodeQuality(q_new, q_select_cost)
                
                randValue = random.random()
                if q_select_quality > thr or randValue > prob: 
                    break 

            vid_start = vid_near           
            q_new = q_select
            q_new_cost = q_select_cost
 
            # add q_new to the tree
            vid_end = self.tree.AddVertex(q_new)
            self.tree.AddEdge(vid_start, vid_end)

            # update max cost 
            self.maxCost = max(self.maxCost, q_new_cost)
            
            # update the distance from the goal, improvement: calculated dist twice
            dist = self.getHCost(q_new, self.goal_config)

            #visualize the updated tree
            if self.visualize:
                self.planning_env.PlotEdge(q_near, q_new) 

        plan = []
        currVertex = vid_end
        while (currVertex != self.tree.GetRootId()):
            currVertex = self.tree.edges[currVertex] 
            plan.insert(0, self.tree.vertices[currVertex]) 
        plan.append(self.goal_config)

        print "tree size: " + str(len(self.tree.vertices))

        return plan

    def getNodeQuality(self, q_near, q_near_cost):

        return 1 - abs(q_near_cost - self.minCost) / (self.maxCost - self.minCost)

    def getHCost(self, config1, config2):
        return numpy.sqrt(sum((config2 - config1)**2)) 
        
    def getGCost(self, currId):
        gCost = 0

        while (currId != self.tree.GetRootId()):
            lastId = currId
            lastConfig = self.tree.vertices[lastId]
            currId = self.tree.edges[currId] 
            currConfig = self.tree.vertices[currId]
            gCost += self.getHCost(currConfig, lastConfig)
        
        return gCost 






#!/usr/bin/env python
'''
PRM package
'''

import numpy as np
import matplotlib.pyplot as plotter
from math import pi
from collisions import PolygonEnvironment
import time
import random
import heapq
import a_star
import rrt

'''
Author: Ryan Farr
Date: 10/13/2016
File: prm.py - Contains all required methods and data structures to create
     and query a PRM. Can be gaussian or regular, can use basic planner
     or RRT planner.

Use: Produce the PRM with test_prm_env method. Query PRM with quer_prm method 

PRM Parameters:
    num_samples - number of samples to create on environment
    step_length - epsilon to be used for collision checking
    env - file path to environment file
    num_neighbors - number of neighbors to try to connect to
    gaussian - whether or not to use gaussian PRM
    rrt_planner - whether or not to use the RRT local planner

PRM Query Parameters:
    pe - the environment to use. Should be returned from test_prm_env
    prm - the PRM to be queried. Should be returend from test_prm_env
    start - the start state
    goal - the goal state
'''

_RRT_PLANNER = 'rrt_planner'
_BASIC_PLANNER = 'basic_planner'

class HeapNode:
    '''
    The node used in a heap in order to find KNN
    '''
    def __init__(self, node, dist):
        self.node = node
        self.dist = dist

    def __cmp__(self, other):
        return cmp(self.dist, other.dist)

class Node:
    '''
    Node used in PRM Graph. Contains a state and connections
    '''
    def __init__(self, state):
        self.state = state
        self.connections = []

    def add_connection(self, node):
        self.connections.append(node)

class PRMGraph:
    '''
    The graph used for PRMs. Contains nodes and edges,
    supports KNN, add_node, add_edge, get_states_and_edges,
    and clone
    '''
    def __init__(self):
        self.nodes = []
        self.edges = []

    def find_k_nearest(self, k, s_query):
        '''
        Gets KNN using heap sort
        '''
        neighbors = []
        for n in self.nodes:
            dist = np.linalg.norm(s_query - n.state)
            if dist > 0:
                hn = HeapNode(n, dist)
                heapq.heappush(neighbors, hn)

        #Check if we have <= k to choose from
        #in which case, simply return that list
        ret_list = []
        if len(neighbors) <= k:
            for k in neighbors:
                ret_list.append(k.node)
            return ret_list
            
        #get the k nearest neighbors
        for i in xrange(k):
            t = heapq.heappop(neighbors)
            ret_list.append(t.node)

        return ret_list

    def add_node(self, node):
        '''
        Add a node to the graph
        '''
        self.nodes.append(node)
        
    def add_edge(self, node1, node2):
        '''
        Add an edge between two nodes in the graph
        '''
        node1.add_connection(node2)
        node2.add_connection(node1)
        self.edges.append((node1.state, node2.state))

    def get_states_and_edges(self):
        '''
        Get the states and edges in a tuple
        Returns (states, edges)
        '''
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

    def clone(self):
        '''
        Returns a clone of the graph which shares no references
        with the original
        '''
        n_graph = PRMGraph()
        mapping = {}
        for node in self.nodes:
            n_n = Node(node.state)
            n_graph.add_node(n_n)
            mapping[node] = n_n

        #add back the edges
        for node in self.nodes:
            for conn in node.connections:
                if conn not in mapping[node].connections:
                    n_graph.add_edge(mapping[node], mapping[conn])

        return n_graph

class PRM:
    def __init__(self, num_samples, num_dimensions, step_length, collision_func, lims,
                  num_neighbors, planner = _BASIC_PLANNER):
        '''
        num_samples - number of times to sample the space
        num_dimensions - dimensionality of space
        step_length - epsilon to use when detecting collisions
        collision_func - function to use for detecting collisions
        lims - the limits on the space
        num_neighbors - how many neighbors to use when connecting the nodes
        planner - basic or rrt planner
        '''
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.in_collision = collision_func
        self.limits = lims
        self.found_path = False
        self.num_neighbors = num_neighbors
        self.planner = planner
        self.attempts = 0

    def clone(self):
        '''
        Clones the PRM. Breaks all references to original PRM
        '''
        n_prm = PRM(self.K, self.n, self.epsilon, self.in_collision, self.limits, self.num_neighbors, self.planner)
        n_prm.T = self.T.clone()
        
        return n_prm

    def build_prm(self, gaussian=False):
        '''
        Builds the PRM graph
        gaussian - whether or not to use gaussian version of PRM
        '''
        self.T = PRMGraph()

        #add all nodes
        for i in xrange(self.K):
            if gaussian:
                self.add_to_tree_gaussian()
            else:
                self.add_to_tree()

        #add all edges
        o_nodes = list(self.T.nodes)
        for node in o_nodes:
            self.attempts += 1
            print self.attempts
            for nb in self.T.find_k_nearest(self.num_neighbors, node.state):
                if not nb in node.connections:
                    self.try_connect(node, nb)
                        
    def query(self, start, goal):
        '''
        Driver function. Queries map to get path from
        start to goal
        start - start state
        goal - goal state
        returns prm, path
        '''
        c = self.clone()
        return c, c._query(start, goal)

    def _query(self, start, goal):
        '''
        Queries current PRM to find path between start and goal.
        Alters the map. Returns path as list of states.
        '''
        #add start and goal to the map
        s_node = Node(start)
        g_node = Node(goal)
        self.T.add_node(s_node)
        self.T.add_node(g_node)
        
        #Add neighbors for the two nodes
        for nb in self.T.find_k_nearest(self.num_neighbors, start):
            self.try_connect(s_node, nb)
                    
        for nb in self.T.find_k_nearest(self.num_neighbors, goal):
            self.try_connect(g_node, nb)

        #Run A* pathfinding to get path
        return a_star.a_star_search(s_node, g_node)

    def add_to_tree(self):
        '''
        Attempt to add one node to the graph
        based on random sample
        '''
        s = self.sample()
        graph_node = Node(s)
        if not self.in_collision(s):
            self.T.add_node(graph_node)

    def add_to_tree_gaussian(self):
        '''
        Attempt to add one node to the tree.
        Randomly samples one point, then samples
        in gaussian around that point. If one is
        in collision and the other is not, keep 
        the one that is not in collision
        '''
        s = self.sample()
        s2 = self.sample_gaussian(s)
        col_1 = self.in_collision(s)
        col_2 = self.in_collision(s2)
        if col_1 and not col_2:
            new_node = Node(s2)
            self.T.add_node(new_node)
        elif col_2 and not col_1:
            new_node = Node(s)
            self.T.add_node(new_node)

    def try_connect(self, q, q_near):
        '''
        Attempt to connect q to q_near
        '''
        if self.in_collision(q.state):
            return None
        
        if self.planner == _BASIC_PLANNER:
            return self.try_connect_basic(q, q_near)
        
        return self.try_connect_rrt(q, q_near)      
    
    def try_connect_basic(self, q, q_near):
        '''
        Attempt to connect q to q_near using basic
        line.
        '''
        #Get the unit vector for the direction to move towards
        q_near_to_q = q.state - q_near.state
        distance = np.linalg.norm(q_near_to_q)
        if distance == 0:
            return 
        q_near_to_q /= distance
        
        #Multiply vector by epsilon so that we can continuously add to q_near and check for collision
        q_near_to_q *= self.epsilon

        q_curr = np.copy(q_near.state)
        while np.linalg.norm(q.state - q_curr) > self.epsilon:
            q_curr += q_near_to_q
            if self.in_collision(q_curr):
                return 

        self.T.add_edge(q, q_near)

    def try_connect_rrt(self, q, q_near):
        '''
        Attempt to connect q to q_near using a RRT
        with start = q and goal = q_near.
        '''
        tree = rrt.RRT(10, self.n, self.epsilon, self.limits, collision_func=self.in_collision)
        plan = tree.build_rrt_connect(q.state, q_near.state)

        #add plan as nodes if we get a plan back
        if plan is not None and len(plan) > 0:
            n = q
            for s in plan:
                new_n = Node(s)
                self.T.add_node(new_n)
                self.T.add_edge(new_n, n)
                n = new_n

            self.T.add_edge(n, q_near)
        
    def sample(self):
        '''
        Sample a new configuration and return
        '''
        point = []
        for dimension in self.limits:
            point.append(self.get_1d_sample(dimension[0], dimension[1]))
        
        return np.array(point)

    def sample_gaussian(self, pt):
        '''
        Sample new configuration using gaussian with mu=pt
        '''
        point = []
        for d in xrange(self.n):
            point.append(self.get_1d_sample_gaussian(pt[d], self.limits[d][0], self.limits[d][1]))

        return np.array(point)

    def get_1d_sample(self, min, max):
        '''
        Returns number between min and max
        '''
        return (random.random()*(max-min))+min

    def get_1d_sample_gaussian(self, mean, min, max):
        '''
        Returns gaussian around mean between with min and max
        as limits
        '''
        val = np.random.normal(mean, 4)
        if val < min:
            return min
        if val > max:
            return max
            
        return val

def test_prm_env(num_samples=500, step_length=2, env='./env0.txt', num_neighbors=5, gaussian=False, rrt_planner=False):
    '''
    Produces PRM
    
    num_samples - number of samples to create on environment
    step_length - epsilon to be used for collision checking
    env - file path to environment file
    num_neighbors - number of neighbors to try to connect to
    gaussian - whether or not to use gaussian PRM
    rrt_planner - whether or not to use the RRT local planner

    returns tuple of environment with PRM: (ep, prm)
    '''
    pe = PolygonEnvironment()
    pe.read_env(env)

    dims = len(pe.start)
    start_time = time.time()
    planner = _RRT_PLANNER if rrt_planner else _BASIC_PLANNER
    
    prm = PRM(num_samples, dims, step_length, pe.test_collisions, pe.lims, num_neighbors, planner)
    prm.build_prm(gaussian)
    
    run_time = time.time() - start_time
    print 'run_time = ', run_time

    pe.draw_plan(None, prm)
    return pe, prm

def query_prm(pe, prm, start=None, goal=None):
    '''
    Queries PRM with start and goal to find a path

    pe - the environment to use. Should be returned from test_prm_env
    prm - the PRM to be queried. Should be returend from test_prm_env
    start - the start state
    goal - the goal state
    
    returns tuple of new PRM with plan: (q_prm, plan)
    '''
    if(start == None):
        start = pe.start
    if(goal == None):
        goal = pe.goal

    q_prm, plan = prm.query(start, goal)
    print 'plan: ', plan

    #Draw correct start and goal
    prev_start = pe.start
    prev_goal = pe.goal
    pe.start = start
    pe.goal = goal
    pe.draw_plan(plan, q_prm)
    pe.start = prev_start
    pe.goal = prev_goal

    return q_prm, plan

def main():
    pe, prm = test_prm_env(num_samples=500, step_length = 2, env='./env0.txt', gaussian=False, rrt_planner=True)
    query_prm(pe, prm)

if __name__ == '__main__':
    main()

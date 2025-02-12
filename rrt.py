#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import matplotlib.pyplot as plotter
from math import pi
from collisions import PolygonEnvironment
import time
import random

'''
Author: Ryan Farr
Date: 10/13/2016
File: rrt.py - Contains all required classes and data structures to create a 
     rapidly-exploring random tree (RRT) based on a given environment.

Use: Generate an environment, run the test_rrt_env method on that environment.
     Set the required parameters.

Parameters:
     num_samples - maximum number of samples to use
     step_length - the epsilon used for collision checking
     env - the file path to the environment defined
     connect - whether or not to use rrt-connect
     bidirectional - whether or not to use bidirectional rrt-connect. NOTE: false if connect is false
'''

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

class TreeNode:
    '''
    Definition for one node in the RRT Search Tree.
    Contains data for the state, children, and parent
    '''
    def __init__(self, state, parent=None):
        self.state = state
        self.children = []
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)

class RRTSearchTree:
    '''
    Definition for the RRT Search Tree. Contains root,
    nodes, and edges. Supports find-nearest, add node,
    get_back_path, and get_states_and_edges.
    '''
    def __init__(self, init):
        self.root = TreeNode(init)
        self.nodes = [self.root]
        self.edges = []

    def find_nearest(self, s_query):
        '''
        Find the node nearest to the query state
        '''
        min_d = 1000000
        nn = self.root
        test = 0
        for n_i in self.nodes:
            d = np.linalg.norm(s_query - n_i.state)
            if d < min_d:
                nn = n_i
                min_d = d
        return (nn, min_d)

    def add_node(self, node, parent):
        self.nodes.append(node)
        self.edges.append((parent.state, node.state))
        node.parent = parent
        parent.add_child(node)

    def get_states_and_edges(self):
        '''
        Returns tuple of all states and edges
        (states, edges)
        '''
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

    def get_back_path(self, n):
        '''
        Returns the path from the given node back to the root. Given as a
        list of states
        '''
        path = []
        while n is not None:
            path.append(n.state)
            n = n.parent

        path.reverse()
        return path

class RRT:
    def __init__(self, num_samples, num_dimensions=2, step_length = 1, lims = None,
                 connect_prob = 0.05, collision_func=None):
        '''
        Initialize an RRT planning instance
        '''
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.connect_prob = connect_prob
        self.goal_node = None

        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = self.fake_in_collision

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in xrange(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)

        self.ranges = self.limits[:,1] - self.limits[:,0]
        self.found_path = False

    def build_rrt(self, init, goal):
        '''
        Build the rrt from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.T = RRTSearchTree(init)
        
        for i in xrange(self.K):
            x_rand = self.sample()
            if self.extend(x_rand) == _REACHED:
                return self.T.get_back_path(self.goal_node)
            
        return None

    def build_rrt_connect(self, init, goal):
        '''
        Build the rrt connect from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.T = RRTSearchTree(init)
        
        for i in xrange(self.K):
            x_rand = self.sample()
            status = self.extend_connect(x_rand, self.T)
            if status[0] == _REACHED:
                return self.T.get_back_path(status[1])

        return None

    def build_rrt_connect_bidirectional(self, init, goal):
        '''
        Build the rrt connect bidirectionally. This builds a tree
        from both the start and the goal.
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False
        self.goal_node_2 = None

        # Build tree and search
        self.T = RRTSearchTree(init)
        self.Tg = RRTSearchTree(goal)

        tree = self.T
        tree2 = self.Tg
        for i in xrange(self.K):
            x_rand = self.sample()
            status = self.extend_connect(x_rand, tree)
            if not status[0] == _TRAPPED:
                status_2 = self.extend_connect(status[1].state, tree2)
                if not status_2[0] == _TRAPPED:
                    return self.get_path(tree, status[1], tree2, status_2[1])
                    
            #switch trees
            if tree == self.T:
                tree  = self.Tg
                tree2 = self.T
            else:
                tree  = self.T
                tree2 = self.Tg

        return None

    def get_path(self, tree1, node1, tree2, node2):
        '''
        Gets the path from goal to start for bidirectional
        RRT-Connect. Takes path from start to connecting node,
        then reverses path from connecting node to goal.
        Returns path as list of states.
        '''
        use_tree1 = tree1
        use_node1 = node1
        use_tree2 = tree2
        use_node2 = node2
        if not tree1 == self.T:
            use_tree1 = tree2
            use_node1 = node2
            use_tree2 = tree1
            use_node2 = node1
                    
        path = use_tree1.get_back_path(use_node1)
        path2 = use_tree2.get_back_path(use_node2)
        path2.reverse()
        path.extend(path2)

        return path

    def sample(self):
        '''
        Sample a new configuration and return
        '''
        # Return goal with connect_prob probability
        if random.random() <= self.connect_prob:
            return self.goal

        point = []
        for dimension in self.limits:
            point.append(self.get_1d_sample(dimension[0], dimension[1]))
        
        return np.array(point)
        
    def get_1d_sample(self, min, max):
        '''
        Sample one dimension given a minimum and maximum value
        '''
        return (random.random()*(max-min))+min


    def extend(self, q):
        '''
        Perform rrt extend operation.
        q - new configuration to extend towards
        '''
        x_near = self.T.find_nearest(q)[0]
        x_near_state = x_near.state
        
        if self.can_connect(q, x_near_state):
            if np.linalg.norm(self.goal - q) <= self.epsilon:
                self.found_path = True
                new_node = TreeNode(self.goal, x_near)
                self.T.add_node(new_node, x_near)
                self.goal_node = new_node
                return _REACHED
            else:
                new_node = TreeNode(q, x_near)
                self.T.add_node(new_node, x_near)
                return _ADVANCED

        return _TRAPPED

    def extend_connect(self, q, tree):
        '''
        Perform rrt extend operation for connect.
        q - new configuration to extend towards
        tree - which tree to extend
        '''
        x_near = tree.find_nearest(q)[0]
        x_near_state = x_near.state
        conn = self.can_connect_C(q, x_near_state, x_near, tree)
        if conn[0]:
            if np.linalg.norm(self.goal - q) <= self.epsilon:
                self.found_path = True
                new_node = TreeNode(self.goal, conn[1])
                tree.add_node(new_node, conn[1])
                self.goal_node = new_node
                return (_REACHED, new_node)
            else:
                new_node = TreeNode(q, conn[1])
                tree.add_node(new_node, conn[1])
                return (_ADVANCED, new_node)

        return (_TRAPPED, None)

    def can_connect_C(self, q, q_near, q_near_node, tree):
        '''
        Determines if q and q_near can be connected for RRT-Connect
        for a given tree.
        '''
        if self.in_collision(q):
            return (False, None)

        #Get the unit vector for the direction to move towards
        q_near_to_q = q - q_near
        distance = np.linalg.norm(q_near_to_q)
        if distance == 0:
            return (False, None)
        q_near_to_q /= distance
        
        #Multiply vector by epsilon so that we can continuously add to q_near and check for collision
        q_near_to_q *= self.epsilon

        #Check for collision
        q_curr = np.copy(q_near)
        parent_node = q_near_node
        while np.linalg.norm(q - q_curr) > self.epsilon:
            q_curr += q_near_to_q
            if self.in_collision(q_curr):
                return (False, None)
            
            new_node = TreeNode(q_curr, parent_node)
            tree.add_node(new_node, parent_node)
            parent_node = new_node
            q_curr = np.copy(q_curr)

        return (True, parent_node)

    def can_connect(self, q, q_near):
        '''
        Tells of q and q_near can be connected using a linear collision check.
        '''
        if self.in_collision(q):
            return False

        #Get the unit vector for the direction to move towards
        q_near_to_q = q - q_near
        distance = np.linalg.norm(q_near_to_q)
        q_near_to_q /= distance
        

        #Multiply vector by epsilon so that we can continuously add to q_near and check for collision
        q_near_to_q *= self.epsilon

        #Check for collision
        q_curr = np.copy(q_near)
        while np.linalg.norm(q - q_curr) > self.epsilon:
            q_curr += q_near_to_q
            if self.in_collision(q_curr):
                return False

        return True

    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False

def test_rrt_env(num_samples=500, step_length=2, env='./env0.txt', start = None, goal = None, connect_prob = 0.05, connect=False, bidirectional=True):
    '''
    create an instance of PolygonEnvironment from a description file and plan a path from start to goal on it using an RRT

    num_samples - number of samples to generate in RRT
    step_length - step size for growing in rrt (epsilon)
    env - path to the environment file to read
    start - the start state
    goal - the goal state
    connect_prob - the probability of choosing the goal to try to connect to rather than a random node
    connect - If True run rrt_connect
    bidirectional - If true and connect is true, uses bidirectional rrt-connect

    returns plan, planner - plan is the set of configurations from start to goal, planner is the rrt used for building the plan
    '''
    pe = PolygonEnvironment()
    pe.read_env(env)

    if start == None:
        start = pe.start
    if goal == None:
        goal = pe.goal

    start.astype(float)
    goal.astype(float)
    pe.start = start
    pe.goal = goal

    dims = len(pe.start)
    start_time = time.time()

    rrt = RRT(num_samples,
              dims,
              step_length,
              lims = pe.lims,
              connect_prob = connect_prob,
              collision_func=pe.test_collisions)
    if connect:
        if bidirectional:
            plan = rrt.build_rrt_connect_bidirectional(pe.start, pe.goal)
        else:
            plan = rrt.build_rrt_connect(pe.start, pe.goal)
    elif bidirectional:
        print '/!\\ ERROR: Can\'t use bidirectional without connect being true. Try again, but use connect = true or change bidirectional to false'
        return None, None
    else:
        plan = rrt.build_rrt(pe.start, pe.goal)
    run_time = time.time() - start_time
    print 'plan:', plan
    print 'run_time =', run_time

    #draw tree
    pe.draw_plan(None, rrt, show_points=False)

    #draw plan
    pe.start = start
    pe.goal = goal
    pe.draw_plan(plan, rrt)
    return plan, rrt

def main():
    s1_0 = np.array([-60., 1.5])
    g1_0 = np.array([39., 133.])
    s2_0 = np.array([-70., 95.])
    g2_0 = np.array([78., 28.5])
    s3_0 = np.array([-19., 46.])
    g3_0 = np.array([1., 30.])

    s1_1 = np.array([0.8, 0.8, 0.8])
    g1_1 = np.array([-0.6, -0.15, -0.3])
    s2_1 = np.array([1.6, 0.9, 0.9])
    g2_1 = np.array([0.6, 0.15, -0.3])
    s3_1 = np.array([3.7, 0.9, 1.0])
    g3_1 = np.array([0.1, 1.3, 1.3])
    test_rrt_env(env='./env1.txt', num_samples=5000, step_length=0.15,start=None, goal=None, connect_prob=0.05, connect=True, bidirectional=True)

if __name__ == "__main__":
    main()

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

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

class TreeNode:
    def __init__(self, state, parent=None):
        self.state = state
        self.children = []
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)

class RRTSearchTree:
    def __init__(self, init):
        self.root = TreeNode(init)
        self.nodes = [self.root]
        self.edges = []

    def find_nearest(self, s_query):
        min_d = 1000000
        nn = self.root
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
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

    def get_back_path(self, n):
        path = []
        while n is not None:
            print 'node: ', n.state, ', with parent: ', n.parent
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
        # Fill me in!
        return None

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

    def can_connect(self, q, q_near):
        if self.in_collision(q):
            return False

        #Get the unit vector for the direction to move towards
        q_near_to_q = q - q_near
        distance = np.linalg.norm(q_near_to_q)
        q_near_to_q /= distance
        

        #Multiply vector by epsilon so that we can continuously add to q_near and check for collision
        q_near_to_q *= self.epsilon

        q_curr = q_near
        while np.linalg.norm(q - q_near) > self.epsilon:
            q_curr += q_near_to_q
            if self.in_collision(q_curr):
                return False

        return True

    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False

def test_rrt_env(num_samples=500, step_length=2, env='./env0.txt', connect=False):
    '''
    create an instance of PolygonEnvironment from a description file and plan a path from start to goal on it using an RRT

    num_samples - number of samples to generate in RRT
    step_length - step size for growing in rrt (epsilon)
    env - path to the environment file to read
    connect - If True run rrt_connect

    returns plan, planner - plan is the set of configurations from start to goal, planner is the rrt used for building the plan
    '''
    pe = PolygonEnvironment()
    pe.read_env(env)

    dims = len(pe.start)
    start_time = time.time()

    rrt = RRT(num_samples,
              dims,
              step_length,
              lims = pe.lims,
              connect_prob = 0.05,
              collision_func=pe.test_collisions)
    if connect:
        plan = rrt.build_rrt_connect(pe.start, pe.goal)
    else:
        plan = rrt.build_rrt(pe.start, pe.goal)
    run_time = time.time() - start_time
    print 'plan:', plan
    print 'run_time =', run_time
    print 'start: ', pe.start
    pe.draw_plan(plan, rrt)
    return plan, rrt

def main():
    test_rrt_env()

if __name__ == "__main__":
    main()

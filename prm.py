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
from rrt import RRT

_RRT_PLANNER = 'rrt_planner'
_BASIC_PLANNER = 'basic_planner'

class Node:
    def __init__(self, state):
        self.state = state
        self.connections = []

    def add_connection(self, node):
        self.connections.append(node)

class PRMGraph:
    def __init__(self):
        self.nodes = []
        self.edges = []

    def find_k_nearest(self, k, s_query):
        neighbors = []
        for n in self.nodes:
            dist = np.linalg.norm(k - n.state)
            if dist > 0:
                heapq.heappush(neighbors, (dist, n))

        ret_list = []
        if len(neighbors) <= k:
            for k in neighbors:
                ret_list.append(k[1])
            return ret_list

        for i in xrange(k):
            t = heapq.heappop(neighbors)
            ret_list.append(t[1])

        return ret_list

    def add_node(self, node):
        self.nodes.append(node)
        
    def add_edge(self, node1, node2):
        node1.add_connection(node2)
        node2.add_connection(node1)
        self.edges.append((node1.state, node2.state))

    def get_states_and_edges(self):
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

class PRM:
    def __init__(self, num_samples, num_dimensions, step_length, collision_func, lims, num_neighbors, planner = _BASIC_PLANNER):
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.in_collision = collision_func
        self.limits = lims
        self.found_path = False
        self.num_neighbors = num_neighbors

    def build_prm(self):
                
        self.T = PRMGraph()

        for i in xrange(self.K):
            self.add_to_tree()

    def build_prm_gaussian(self):
        return None

    def query(self, start, goal):
        #take in start and goal, produce a plan using the curent PRM
        return None 

    def add_to_tree(self):
        s = self.sample()
        graph_node = Node(s)
        if not self.in_collision(s):
            self.T.add_node(graph_node)
            for node in self.T.find_k_nearest(self.num_neighbors, s):
                if self.can_connect(s, node.state):
                    self.T.add_edge(graph_node, node)

    def can_connect(self, q, q_near):
        if self.in_collision(q):
            return False

        #Get the unit vector for the direction to move towards
        q_near_to_q = q - q_near
        distance = np.linalg.norm(q_near_to_q)
        q_near_to_q /= distance
        

        #Multiply vector by epsilon so that we can continuously add to q_near and check for collision
        q_near_to_q *= self.epsilon

        q_curr = np.copy(q_near)
        while np.linalg.norm(q - q_curr) > self.epsilon:
            q_curr += q_near_to_q
            if self.in_collision(q_curr):
                return False

        return True

    def sample(self):
        '''
        Sample a new configuration and return
        '''
        point = []
        for dimension in self.limits:
            point.append(self.get_1d_sample(dimension[0], dimension[1]))
        
        return np.array(point)
        
    def get_1d_sample(self, min, max):
        return (random.random()*(max-min))+min

def test_prm_env(num_samples=500, step_length=2, env='./env0.txt', num_neighbors = 3, gaussian=False, rrt_planner=False):
    pe = PolygonEnvironment()
    pe.read_env(env)

    dims = len(pe.start)
    start_time = time.time()
    
    prm = PRM(num_samples, dims, step_length, pe.test_collisions, pe.lims, num_neighbors)
    prm.build_prm()
    
    run_time = time.time() - start_time
    print 'run_time = ', run_time

    pe.draw_plan(None, prm)
    return prm

def main():
    test_prm_env()

if __name__ == '__main__':
    main()

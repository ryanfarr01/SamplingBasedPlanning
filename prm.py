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
from rrt import RRT

_RRT_PLANNER = 'rrt_planner'
_BASIC_PLANNER = 'basic_planner'

class PRM:
    def __init__(self, num_samples, num_dimensions, step_length, collision_func, lims, planner = _BASIC_PLANNER):
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.in_collision = collision_func
        self.limits = lims

    def build_prm():
        return None

    def query(start, goal):
        #take in start and goal, produce a plan using the curent PRM
        return None
    
    #def 

def test_prm_env(num_samples=500, step_length=2, env='./env0.txt', gaussian=False, rrt_planner=False):
    pe = PolygonEnvironment()
    pe.read_env(env)

    dims = len(pe.start)
    start_time = time.time()
    
    prm = PRM()

    run_time = time.time() - start_time
    print 'run_time = ', run_time
    return prm
    

def main():
    test_prm_env()

if __name__ == '__main__':
    main()

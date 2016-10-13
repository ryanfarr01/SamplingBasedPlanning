#!/usr/bin/env python

from prm import Node
import numpy as np
import heapq

'''
Author: Ryan Farr
Date: 10/13/2016
File: a_star.py - contains a_star_search method which takes start and goal nodes and finds a path
    between them. Returns None if no path exists. Both start and goal must already be nodes
    in an existing graph. The nodes must have a variable 'state' that gives their location in
    n-dimensional space.

Use: import a_star, create the graph, pass start and goal nodes to a_star_search method
'''

def a_star_search(start, goal):
    '''
    Performs A* search to find a path to the goal. Start and goal should
    both be nodes already in a pre-made graph.

    Returns a path or None of one does not exist

    start - the start state
    goal - the goal state
    '''

    frontier = PriorityQ()
    start_node = SearchNode(start.state, start)
    goal_node = SearchNode(goal.state, goal)
    visited = set()
    frontier.push(start_node, 0)
    while len(frontier) > 0:
        n_i = frontier.pop()
        if not in_list(n_i, visited):
            visited.add(n_i)
            if is_goal(n_i, goal_node):
                return get_path(n_i)
            else:
                for nb in n_i.node.connections:
                    s_prime = nb
                    n_prime = SearchNode(s_prime.state, s_prime, n_i, n_i.cost + np.linalg.norm(n_i.state - s_prime.state))
                    tot_cost = n_prime.cost + h(n_prime.state, goal_node.state)
                    if (n_prime not in frontier) or (tot_cost < frontier.get_cost(n_prime)):
                        frontier.push(n_prime, tot_cost)
    return None

def is_goal(node, goal):
    '''
    Returns true if the node is the goal. False otherwise
    '''

    return node.node == goal.node

def h(state1, goal):
    '''
    Returns a float value for the minimum distance
    between the state and goal. Acts as a heuristic
    '''

    return np.linalg.norm(state1 - goal)

def get_path(node):
    '''
    Gets the path from this node to the start. Returns a list
    of states as the path
    '''
    
    path = []
    while node is not None:
        path.append(node.state)
        node = node.parent

    path.reverse()
    return path

def in_list(node, visited):
    '''
    Returns true of the node is in the list. False otherwise
    '''
    for v in visited:
        if node.__eq__(v):
            return True
    return False

class SearchNode:
    def __init__(self, s, node, parent=None, cost=0):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''
        self.parent = parent
        self.cost = cost
        self.state = s
        self.node = node

    def __str__(self):
        '''
        Return a human readable description of the node
        '''
        return str(self.state)

    def __hash__(self):
        return hash(repr(self))

    def __eq__(self, other):
        i = 0
        for d in self.state:
            if not self.state[i] == other.state[i]:
                return False
            i += 1
        return True

    def __ne__(self, other):
        return not self.__eq__(other)

class PriorityQ:
    '''
    Priority queue implementation with quick access for membership testing
    Setup currently to only with the SearchNode class
    '''
    def __init__(self):
        '''
        Initialize an empty priority queue
        '''
        self.l = [] # list storing the priority q
        self.s = set() # set for fast membership testing

    def __contains__(self, x):
        '''
        Test if x is in the queue
        '''
        for n in self.s:
            if self.is_equal(n.state, x.state):
                return True

        return False

    def is_equal(self, state1, state2):
        for i in xrange(len(state1)):
            if not state1[i] == state2[i]:
                return False
        return True

    def push(self, x, cost):
        '''
        Adds an element to the priority queue.
        If the state already exists, we update the cost
        '''
        if x in self.s:
            return self.replace(x, cost)
        heapq.heappush(self.l, (cost, x))
        self.s.add(x)

    def pop(self):
        '''
        Get the value and remove the lowest cost element from the queue
        '''
        x = heapq.heappop(self.l)
        self.s.remove(x[1])
        return x[1]

    def peak(self):
        '''
        Get the value of the lowest cost element in the priority queue
        '''
        x = self.l[0]
        return x[1]

    def __len__(self):
        '''
        Return the number of elements in the queue
        '''
        return len(self.l)

    def replace(self, x, new_cost):
        '''
        Removes element x from the q and replaces it with x with the new_cost
        '''
        for y in self.l:
            if x.state == y[1].state:
                self.l.remove(y)
                self.s.remove(y[1])
                break
        heapq.heapify(self.l)
        self.push(x, new_cost)

    def get_cost(self, x):
        '''
        Return the cost for the search node with state x.state
        '''
        for y in self.l:
            if self.is_equal(x.state, y[1].state):
                return y[0]

    def __str__(self):
        '''
        Return a string of the contents of the list
        '''
        return str(self.l)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Alexandre Justo Miro}
# {19970907-T212}
# {aljm@kth.se}

from dubins import *
from math import sqrt, atan2
import random

# NODE (Build Nodes)
class Node:

    def __init__(self, x=None, y=None, theta=None, phi=None, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.parent = parent
        return

# COST (Calculate the cost function or heuristics of two arbitrary nodes n1 and n2)
def cost(n1, n2):
    return sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)

# OBSTACLES (Test whether a node hits an obstacle or penetrates within a safety distance around an obstacle)
def obstacles(x, y, car):

    safety_term_o = 0.2

    for obs_one in car.obs:
        if sqrt((obs_one[0] - x)**2 + (obs_one[1] - y)**2) < obs_one[2] + safety_term_o:
            return True
    return False

# BOUNDS (Test whether a node is out of bounds or near that)
def bounds(x, y, xlb, xub, ylb, yub):

    safety_term_b = 0.2

    if (x < xlb + safety_term_b) or (x > xub - safety_term_b) or (y < ylb + safety_term_b) or (y > yub - safety_term_b):
        return True
    else:
        return False

# SOLUTION (Main loop)
def solution(car):

    new_nodes = [ Node(car.x0, car.y0, 0) ]

    times = []
    controls = []

    dt = 0.7

    it = 0

    while it < 20000:
        
        it += 1
        
        # If the latest node is not in target yet, try a new random node
        if (new_nodes[-1].x - car.xt)**2 + (new_nodes[-1].y - car.yt)**2 > 1.5**2:
            x = random.uniform(car.xlb+2, car.xub-0.5)
            y = random.uniform(car.ylb+0.1, car.yub-0.1)
            theta = random.uniform(-pi, pi)
            current_node =  Node(x, y, theta)
            next_node = new_nodes[-1]
          
            # Pick the node with the lowest cost function
            cheapest_cost = cost(next_node, current_node) 
            for test_node in new_nodes:
                test_cost = cost(test_node, current_node) 
                if test_cost <= cheapest_cost:
                    cheapest_cost = test_cost
                    next_node = test_node

            # Compute the control angle phi from the attributes of the node picked. It must be between -pi/4 and pi/4
            phi = atan2(y - next_node.y, x - next_node.x) - next_node.theta 
            if phi > pi/4:
                phi = pi/4
            if phi < -pi/4:
                phi = -pi/4

            # Compute the dynamics of the vehicle from "t" to "t+1" and get a candidate node or "child"
            xn, yn, thetan = step(car, next_node.x, next_node.y, next_node.theta, phi, dt)
            child = Node(xn, yn, thetan, phi, next_node)

            # Check whether the "child" is suitable
            if (obstacles(child.x, child.y, car) == False): 
                if not(bounds(child.x, child.y, car.xlb, car.xub, car.ylb, car.yub)):
                    new_nodes.append(child)
            continue

        else:
            break 

    # After the while loop ends, hopefully a node has reached the target
    succeed_node = [new_nodes[-1]]
    
    while True:
        i = succeed_node[-1].parent
        if i != None:
            succeed_node.append(i)
        else:
            break

    for counter in range(2, len(succeed_node) + 1):
        controls.append(succeed_node[-counter].phi)
    for counter in range(0, len(controls) + 1):
        times.append(counter * dt)

    return controls, times

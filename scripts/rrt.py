#!/usr/bin/env python

import numpy as np
import rospy
import HelperFunctions as hf
import scipy.spatial as spatial

def sample_c_space(num_samples):
    lower, upper = hf.get_limits()
    q = np.zeros((len(hf.frame_dict), num_samples))
    for i, joint in enumerate(hf.frame_dict.keys()):
        q[i,:] = (np.random.uniform(lower[joint], upper[joint], num_samples))
    return q.T # n x 7

def create_c_space(q_samples):
    tree = spatial.KDTree(q_samples)
    return tree

def get_nearest_neighbor(c_space, q):
    _, idx = c_space.query(q)
    return c_space.data[idx]
    

def check_collision(q_new):
    return True

def check_path_collision(q_near, q_new):
    return False

class Tree():
    def __init__(self, index, children, parent):
        self.index = index
        self.children = children
        self.parent = parent

    def find(self, index):
        if self.index = index:
            return self
        else:
            for child in children:
                return child.find(index)
        return False
    
    def add_child(self, index)
        self.children.append(Tree(index, [], self))        
    
    

class RRT():
    EPSILON = 0.01

    def __init__(self, q_init):
        self.vertices = spatial.KDTree(q_init)
        self.tree = Tree(0, [], None)

    def nearest_neighbor(self, q):
        _, idx = self.vertices.query(q)
        return idx, np.array(self.vertices.data[idx])

    def new_config(self, q, q_near): 
        # make motion toward q with some fixed distance and test for collision
        relpos = q - q_near
        dist = np.norm(relpos)
        if dist < self.EPSILON:
            q_new = q
        else:
            movedir = relpos/dist
            q_new = q_near + movedir * self.EPSILON
        if check_path_collision(q_near, q_new):
            return None
        else:
            return q_new

    def add_vertex_and_edge(self, q_near_idx, q_new):
        q_new_idx = len(self.vertices.data)
        new_data = self.vertices.data.append(q_new.tolist())
        self.vertices = spatial.KDTree(newdata)
        q_near_node = self.tree.find(q_near_idx)
        q_near_node.add_child(q_new_idx)

    def extend(self, q):
        q_near_idx, q_near = self.nearest_neighbor(q)
        q_new = self.new_config(q, q_near)
        if q_new:
            self.add_vertex_and_edge(q_near_idx, q_new)
            if q_new == q:  
                return q_new, "Reached"
            else:
                return q_new, "Advanced"
        else:
            return q_new, "Trapped"

    def connect(self, q):
        _, status = self.extend(q)
        while status == "Advanced":
            _, status = self.extend(q)
        return s

    def get_max_index(self):
        return len(self.vertices.data) - 1

    def get_config(self, index):
        ''' return configuration of node with index'''
        return rrt.vertices.data[index]
            

def plan_rrt_connect(q_init, q_goal):
    Ta = RRT(q_init)
    Tb = RRT(q_goal)
    for _ in range(self.MAX_STEPS):
        q_rand = sample_c_space(1)
        q_new, status = Ta.extend(q_rand)
        if status != "Trapped":
            if Tb.connect(q_new) == "Reached":
                return get_path_connect(Ta, Tb, q_init, q_goal)
        Ta, Tb = Tb, Ta
    return "Failure"

def get_path_connect(Ta, Tb, q_init, q_goal):
    path_a = get_path(Ta)
    path_b = get_path(Tb)
    if path_a[-1] == q_init):
        path_a.reverse()
        return path_a + path_b
    else:
        path_b.reverse()
        return path_b + path_a
    
def get_path(rrt):
    ''' Return list of configurations from most recently added leaf node to root'''
    path = []
    golden_spike_idx = rrt.get_max_index()
    node = rrt.tree.find(start_idx)
    while node.parent:
        path.append(rrt.get_config(node.index))
        node = node.parent
    return path
    
        

if __name__ == '__main__':
    q =  sample_c_space(10)
    print q
    tree = create_c_space(q)

    #print get_nearest_neighbor(tree, sample_c_space(1))
    print tree.data


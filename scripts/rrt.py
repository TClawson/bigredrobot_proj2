#!/usr/bin/env python

import numpy as np
import rospy
import HelperFunctions as hf
import scipy.spatial as spatial
from collision_checker.srv import *
from std_msgs.msg import String

import pylab
import matplotlib.pyplot as plt
import matplotlib.patches as patches



def sample_c_space(num_samples):
    lower, upper = hf.get_limits()
    q = np.zeros((len(hf.frame_dict), num_samples))
    for i, joint in enumerate(hf.frame_dict.keys()):
        q[i,:] = (np.random.uniform(lower[joint], upper[joint], num_samples))
    return q.T # n x 7

def sample_c_space_2d(num_samples):
    lower, upper = hf.get_limits()
    q = np.zeros((2, num_samples))
    for i, joint in enumerate(hf.frame_dict.keys()):
        q[i,:] = (np.random.uniform(lower[joint], upper[joint], num_samples))
        if i == 1:
            break
    return q.T # n x 2

def create_c_space(q_samples):
    tree = spatial.KDTree(q_samples)
    return tree

def get_nearest_neighbor(c_space, q):
    _, idx = c_space.query(q)
    return c_space.data[idx]
    

def check_collision(q):
#    return (q[0] < 0.6 and q[0] > 0.4) and (q[1] < 0.6 and q[1] > 0.4)
#    return False
    try:
        checker = rospy.ServiceProxy('check_collision', CheckCollision)
        response = checker(String('left'), q.tolist())
    except rospy.ServiceException, e:
        print e
    return response.collision

def check_path_collision(q_init, q_goal, epsilon=0.01):
    if check_collision(q_goal):
        return True    
    relpos = q_goal - q_init
    dist_to_goal = np.linalg.norm(relpos)
    movedir = relpos/dist_to_goal
    q = q_init
    while dist_to_goal > epsilon:
        q = q + epsilon*movedir
        dist_to_goal = np.linalg.norm(q_goal - q)        
        if check_collision(q):
            return True
    return False

class Tree():
    def __init__(self, index, children, parent):
        self.index = index
        self.children = children
        self.parent = parent

    def find(self, index):
        if self.index == index:
            return self
        else:
            for child in self.children:
                result = child.find(index)
                if result != False:
                    return result
        return False
    
    def add_child(self, index):
        self.children.append(Tree(index, [], self))        
    
    def __repr__(self, level=0):
        ret = "\t"*level+repr(self.index)+"\n"
        for child in self.children:
            ret += child.__repr__(level+1)
        return ret
    

class RRT():

    def __init__(self, q_init):
        self.vertices = spatial.KDTree([q_init])
        self.tree = Tree(0, [], None)

    def nearest_neighbor(self, q):
        _, idx = self.vertices.query(q)
        return idx, np.array(self.vertices.data[idx,:]).squeeze()

    def new_config(self, q, q_near, epsilon): 
        # make motion toward q with some fixed distance and test for collision
        relpos = q - q_near
        dist = np.linalg.norm(relpos)
        if dist < epsilon:
            q_new = q
        else:
            movedir = relpos/dist
            q_new = q_near + movedir * epsilon
        if check_path_collision(q_near, q_new):
            return None
        else:
            return q_new

    def add_vertex_and_edge(self, q_near_idx, q_new):
        q_new_idx = len(self.vertices.data)
        new_data = np.append(self.vertices.data, [q_new], axis=0)
        self.vertices = spatial.KDTree(new_data)
        q_near_node = self.tree.find(q_near_idx)
        try:        
            q_near_node.add_child(q_new_idx)
        except Exception:
            print q_near_node
            print self.tree
            print q_near_idx
            raise ValueError
    def extend(self, q, epsilon):
        q_near_idx, q_near = self.nearest_neighbor(q)
        q_new = self.new_config(q, q_near, epsilon)
        if q_new is not None:
            self.add_vertex_and_edge(q_near_idx, q_new)
            if (q_new == q).all():  
                return q_new, "Reached"
            else:
                return q_new, "Advanced"
        else:
            return q_new, "Trapped"

    def connect(self, q, epsilon):
        _, status = self.extend(q, epsilon)
        while status == "Advanced":
            _, status = self.extend(q, epsilon)
        return status

    def get_max_index(self):
        return len(self.vertices.data) - 1

    def get_config(self, index):
        ''' return configuration of node with index'''
        return self.vertices.data[index]
            

def plan_rrt_connect(q_init, q_goal, epsilon=0.01, max_steps=300):
    Ta = RRT(q_init)
    Tb = RRT(q_goal)
    for _ in range(max_steps):
        q_rand = sample_c_space(1).squeeze()
        q_new, status = Ta.extend(q_rand, epsilon)
        if status != "Trapped":
            if Tb.connect(q_new, epsilon) == "Reached":
                return get_path_connect(Ta, Tb, q_init, q_goal)
        Ta, Tb = Tb, Ta
    return None

def get_path_connect(Ta, Tb, q_init, q_goal):
    path_a = get_path(Ta)
    path_b = get_path(Tb)
    if (path_a[-1] == q_init).all():
        path_a.reverse()
        path_a.pop()
        return np.array(path_a + path_b)
    else:
        path_b.reverse()
        path_b.pop()
        return np.array(path_b + path_a)
    
def get_path(rrt):
    ''' Return list of configurations from most recently added leaf node to root'''
    path = []
    golden_spike_idx = rrt.get_max_index()
    node = rrt.tree.find(golden_spike_idx)
    while node.parent:
        path.append(rrt.get_config(node.index))
        node = node.parent
    path.append(rrt.get_config(node.index))
    return path
    
def smooth_path(path, max_iter=20):
    for _ in range(max_iter):
        pathlen = len(path)
        start_idx = np.random.randint(0,pathlen)
        end_idx = np.random.randint(0,pathlen)   
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx
        start = path[start_idx]
        end = path[end_idx]
        if not check_path_collision(start, end):
            path_a = path[:start_idx+1]
            path_b = path[end_idx:]
            print 'path a:', path_a
            print 'path b:', path_b
            path = np.concatenate([path_a, path_b])
    return path

if __name__ == '__main__':

    q_init = np.array([0, 0])
    q_goal = np.array([1, 1])
    #print q_init, q_goal
    result = plan_rrt_connect(q_init, q_goal, epsilon=0.03)
    smooth = smooth_path(result)

    pylab.plot(np.array(result)[:,0], np.array(result)[:,1], marker='+', linestyle='-') 
    plt.axes().add_patch(patches.Rectangle((0.4, 0.4), 0.2, 0.2, fill = False))
    plt.axes().plot(np.array(smooth)[:,0], np.array(smooth)[:,1], marker='+', color='r', linestyle='-')    
    plt.axes().set_aspect('equal', 'datalim')
    
    pylab.show()

    


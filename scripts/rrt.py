#!/usr/bin/env python

import numpy as np
import rospy
import helper_functions as hf
import scipy.spatial as spatial
from collision_checker.srv import *
from std_msgs.msg import String

import pylab
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def sample_c_space(num_samples):
    '''
    Returns num_samples of random samples uniformly distributed in c-space
    '''
    lower, upper = hf.get_limits()
    q = np.zeros((len(hf.frame_dict), num_samples))
    for i, joint in enumerate(hf.frame_dict.keys()):
        q[i,:] = (np.random.uniform(lower[joint], upper[joint], num_samples))
    return q.T # n x 7

def sample_c_space_2d(num_samples):
    '''
    Deprecated - used for testing
    '''
    lower, upper = hf.get_limits()
    q = np.zeros((2, num_samples))
    for i, joint in enumerate(hf.frame_dict.keys()):
        q[i,:] = (np.random.uniform(lower[joint], upper[joint], num_samples))
        if i == 1:
            break
    return q.T # n x 2

def check_collision(q):
    '''
    Service call to collision checker
    '''
    try:
        checker = rospy.ServiceProxy('check_collision', CheckCollision)
        response = checker(String('left'), q.tolist())
    except rospy.ServiceException, e:
        print e
    return response.collision

def check_path_collision(q_init, q_goal, epsilon=0.01):
    '''
    Checks if a line segment is valid by checking for collisions every epsilon
    '''
    # NOTE: Chose default epsilon=0.01 radians because it equates to roughly 1 cm end effector movement at baxter's full arm extension
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
    '''
    Simple tree class. Leaves contain index values.
    '''
    def __init__(self, index, children, parent):
        self.index = index
        self.children = children
        self.parent = parent

    def find(self, index):
        '''
        Find the leaf with the specified index value
        '''
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
    '''
    Rapidly-exploring Random Tree.
    '''
    def __init__(self, q_init):
        self.vertices = spatial.KDTree([q_init])
        self.tree = Tree(0, [], None)

    def nearest_neighbor(self, q):
        '''
        Returns the index and point corresponding to the nearest neighbor to q using euclidean distance
        '''
        _, idx = self.vertices.query(q)
        return idx, np.array(self.vertices.data[idx,:]).squeeze()

    def new_config(self, q, q_near, epsilon): 
        '''
        Make motion toward q with some fixed distance and test for collision
        '''
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
        # Index value of the new node
        q_new_idx = len(self.vertices.data)
        # New array of node points
        new_data = np.append(self.vertices.data, [q_new], axis=0)
        # Update self.vertices with new KDTree
        self.vertices = spatial.KDTree(new_data)
        # Find the node corresponding to the index of nearest-neighbor
        q_near_node = self.tree.find(q_near_idx)
        try:        
            q_near_node.add_child(q_new_idx)
        except Exception:
            raise ValueError

    def extend(self, q, epsilon):
        '''
        Extend the tree towards point q by epsilon
        '''
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
        '''
        Attempts to connect the tree to point q, extending by increments of epsilon
        '''
        _, status = self.extend(q, epsilon)
        while status == "Advanced":
            _, status = self.extend(q, epsilon)
        return status

    def get_max_index(self):
        return len(self.vertices.data) - 1

    def get_config(self, index):
        ''' 
        Return configuration of node with index
        '''
        return self.vertices.data[index]
            

def plan_rrt_connect(q_init, q_goal, epsilon=0.01, max_steps=300):
    '''
    Plans a path using (bi-directional) RRT-Connect algorithm between q_init and q_goal
    '''
    if check_collision(q_goal):
        rospy.logerr("Goal configuration is in collision, no path possible")
        return None
    elif check_collision(q_init):
        rospy.logerr("Initial configuration is in collision, no path possible")
        return None
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
    '''
    Returns path between q_init and q_goal using trees Ta, Tb
    '''
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
    ''' 
    Return list of configurations from most recently added leaf node to root
    '''
    path = []
    golden_spike_idx = rrt.get_max_index()
    node = rrt.tree.find(golden_spike_idx)
    while node.parent:
        path.append(rrt.get_config(node.index))
        node = node.parent
    path.append(rrt.get_config(node.index))
    return path
    
def smooth_path(path, max_iter=20):
    '''
    Smooths path by attempting to randomly connect points max_iter times
    '''
    point_ids = range(len(path))
    checked_pairs = []
    for i in range(max_iter):
        pathlen = len(path)
        start_idx = np.random.randint(0,pathlen)
        end_idx = np.random.randint(0,pathlen)  
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx
        start = path[start_idx]
        end = path[end_idx]
        # Update checked pairs list
        start_id = point_ids[start_idx]
        end_id = point_ids[end_idx]
        # Don't try to connect the same point, adjacent points, or previously connected points
        if abs(start_idx - end_idx) <= 1 or (start_id, end_id) in checked_pairs:
            print 'Bad Pair', i, len(path)
            continue
        checked_pairs.append((start_id, end_id))
        # If it's valid, create the new path
        if not check_path_collision(start, end):
            path_a = path[:start_idx+1]
            path_b = path[end_idx:]
            ids_a = point_ids[:start_idx+1]
            ids_b = point_ids[end_idx:]
            path = np.concatenate([path_a, path_b])
            point_ids = ids_a + ids_b
    return path

if __name__ == '__main__':
    # NOTE: For testing
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

    


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
    

if __name__ == '__main__':
    q =  sample_c_space(10)
    tree = create_c_space(q)

    print get_nearest_neighbor(tree, sample_c_space(1))
        


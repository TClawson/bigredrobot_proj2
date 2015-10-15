#!/usr/bin/env python

import numpy as np
import tf
import rospy
import geometry_msgs.msg


#NOTE: Pass in all transforms relative to origin O_0

def get_transforms(limb):
    listener = tf.TransformListener()
    frames = limb.joint_names
    transforms = []
    for frame in frames:
        try:
            (trans,rot) = listener.lookupTransform(frame, 'base', rospy.Time()) 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Bad things have happened re: joint frame transforms')
        transforms.append((trans,rot))
    return transforms
   


def jacobian(transforms):
    '''
    Returns Jacobian from an input list of (trans, rot) for each frame
    '''   
    # Utility functions
    ros_to_np_q = lambda q: np.array([q.x, q.y, q.z, q.w])
    q_mult = tf.transformations.quaternion_multiply
    q_conj = tf.transformations.quaterion_conjugate
        
    # Convert ROS quaternions to 4-element numpy arrays
    np_quaternions = [ros_to_np_q(q) for (_, q) in transforms] 

    # Find z axis vector for each frame
    z_base = np.array([0,0,1,0]) # z-axis in base frame (as quaternion)
    z_vectors = []
    for q in np_quaternions:
        z_quaternion = q_mult(q_mult(q, z_base), q_conj(q)) #p' = qpq^-1
        z_vectors.append(z_quaternion[0:3])

    J = numpy.empty(6,(len(transforms)))
    for j, (frame_origin, _) in enumerate(transforms):
         for i in range(3):
            J[i,j] = np.cross(z_vectors[i], frame_origin) # z_i * (O_n - O_i)
         for i in range(3,6):
            J[i,j] = z_vectors[i] # z_i
    
    return J


# q' = J^+ xi + (I - J^+ J)b



def rpinv(J):
    '''
    Right pseudo inverse
    '''
    return J.T * np.linalg.inv(J * J.T)

def lpinv(J):
    '''
    Left pseudo inverse
    '''
    return np.linalg.inv(J.T * J) * J.T

def find_joint_vels(J,ee_vels,b):
    '''
    Find joint velocity vector corresponding to a given end effector velocity vector
    ee_vels using Jacobian J and cost function b.
    '''
    A = rpinv(J)*J
    I = np.identity(max(A.shape))
    return lpinv(J) * ee_vels + (I - A)*b


    

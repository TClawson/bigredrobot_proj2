#!/usr/bin/env python

import numpy as np
import tf
import rospy
import geometry_msgs.msg
import rospkg
import xml.etree.ElementTree as ET
from scipy import interpolate


#NOTE: Pass in all transforms relative to origin O_0

frame_dict = {  "left_s0": "left_upper_shoulder",
                "left_s1": "left_lower_shoulder",
                "left_e0": "left_upper_elbow",
                "left_e1": "left_lower_elbow",
                "left_w0": "left_upper_forearm",
                "left_w1": "left_lower_forearm",
                "left_w2": "left_wrist"}

PLANE_FILE = '/scripts/plane.txt'
GOAL_FILE = '/scripts/goal.txt'

def get_limits():
    rospack = rospkg.RosPack()
    path = rospack.get_path("baxter_description")
    tree = ET.parse(path + "/urdf/baxter.urdf")
    root = tree.getroot()

    lower_limits = {}
    upper_limits = {}

    for joint_name in frame_dict.keys():
        joint_element = root.find(".//joint[@name='%s']" %(joint_name))
        limit_element = joint_element.find('limit')        
        lower_limits[joint_name] = float(limit_element.get('lower'))
        upper_limits[joint_name] = float(limit_element.get('upper'))
    
    return lower_limits, upper_limits

def get_joint_info(joint_angles, joint_limits):
    '''
    Return the current position and lower, upper limits of each joint
    '''
    joint_info = {}
    for joint_name in frame_dict.keys():
        joint_info[joint_name] = joint_angles[joint_name], joint_limits[joint_name][0], joint_limits[joint_name][1]

    return joint_info
    

def get_transforms(listener, limb):
    joints = limb.joint_names()
    transforms = []
    for joint in joints:
        #magic!
        frame = frame_dict[joint]
        try:
            (trans,rot) = listener.lookupTransform(frame, 'base', rospy.Time(0)) 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(e)
            rospy.logwarn('Bad things have happened re: joint frame transforms. Frame: %s' %(frame))
        transforms.append((trans,rot))
    return transforms
   


def jacobian(transforms):
    '''
    Returns Jacobian from an input list of (trans, rot) for each frame
    '''   
    # Utility functions
    ros_to_np_q = lambda q: np.array(q)
    q_mult = tf.transformations.quaternion_multiply
    q_conj = tf.transformations.quaternion_conjugate
        
    # Convert ROS quaternions to 4-element numpy arrays
    np_quaternions = [ros_to_np_q(q) for (_, q) in transforms] 

    # Find z axis vector for each frame
    z_base = np.array([0,0,1,0]) # z-axis in base frame (as quaternion)
    z_vectors = []
    for q in np_quaternions:
        z_quaternion = q_mult(q_mult(q, z_base), q_conj(q)) #p' = qpq^-1
        z_vectors.append(z_quaternion[0:3])

    J = np.empty((6, len(transforms)))
    for i, (frame_origin, _) in enumerate(transforms):
        J[0:3,i] = np.cross(z_vectors[i], frame_origin) # z_i * (O_n - O_i)
        J[3:6,i] = z_vectors[i] # z_i
    
    return J


# q' = J^+ xi + (I - J^+ J)b



def rpinv(J):
    '''
    Right pseudo inverse
    '''
    return np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))

def lpinv(J):
    '''
    Left pseudo inverse
    '''
    return np.dot(np.linalg.inv(np.dot(J.T, J)), J.T)

def find_joint_vels(J,ee_vels,b):
    '''
    Find joint velocity vector corresponding to a given end effector velocity vector
    ee_vels using Jacobian J and cost function b.
    '''
    A = rpinv(J)*J
    I = np.identity(max(A.shape))
    return lpinv(J) * ee_vels + (I - A)*b

def get_plane(p1, p2, p3):
    '''
    Returns a unit vector normal to the 3 input points    
    '''
    r12 = p2 - p1
    r32 = p2 - p3
    normal = np.matrix(np.cross(r12.A1, r32.A1)).T
    normal = normal/np.linalg.norm(normal)
    return p1, normal

def project_point(point, normal, q):
    '''
    Projects q into the plane defined by normal, point
    '''
    return q - np.dot(q.A1 - point.A1, normal.A1) * normal

def save_plane(point, normal):
    '''
    Saves plane points to file
    '''
    rospack = rospkg.RosPack()
    path = rospack.get_path("bigredrobot_proj2")
    f = open(path+PLANE_FILE, 'w')
    f.write(str(point.item(0)) + ":" + str(point.item(1)) + ":" + str(point.item(2)) + '\n')
    f.write(str(normal.item(0)) + ":" + str(normal.item(1)) + ":" + str(normal.item(2)) + '\n')
    f.close()

def save_goal(start, goal):
    '''
    Saves plane points to file
    '''
    rospack = rospkg.RosPack()
    path = rospack.get_path("bigredrobot_proj2")
    f = open(path+GOAL_FILE, 'w')
    f.write(str(start.item(0)) + ":" + str(start.item(1)) + ":" + str(start.item(2)) + '\n')
    f.write(str(goal.item(0)) + ":" + str(goal.item(1)) + ":" + str(goal.item(2)) + '\n')
    f.close()


def load_plane():
    '''
    Reads points from plane file and returns the plane
    '''
    rospack = rospkg.RosPack()
    path = rospack.get_path("bigredrobot_proj2")
    f = open(path+PLANE_FILE, 'r')

    #first line is point
    line = f.readline()
    coords = [x.strip() for x in line.split(':')]
    point = np.matrix([float(coords[0]),float(coords[1]),float(coords[2])]).T


    #second line is norm
    line = f.readline()
    coords = [x.strip() for x in line.split(':')]
    norm = np.matrix([float(coords[0]),float(coords[1]),float(coords[2])]).T

    f.close()

    return point, norm

def load_goal():
    '''
    Reads points from plane file and returns the plane
    '''
    rospack = rospkg.RosPack()
    path = rospack.get_path("bigredrobot_proj2")
    f = open(path+GOAL_FILE, 'r')

    #first line is point
    line = f.readline()
    coords = [x.strip() for x in line.split(':')]
    start = np.matrix([float(coords[0]),float(coords[1]),float(coords[2])]).T


    #second line is norm
    line = f.readline()
    coords = [x.strip() for x in line.split(':')]
    goal = np.matrix([float(coords[0]),float(coords[1]),float(coords[2])]).T

    f.close()

    return start, goal

def get_spline(p1, p2, p3, p4):
    x = np.array([p1[0,0], p2[0,0], p3[0,0], p4[0,0]])
    y = np.array([p1[1,0], p2[1,0], p3[1,0], p4[1,0]])
    z = np.array([p1[2,0], p2[2,0], p3[2,0], p4[2,0]])
    rospy.logwarn(x)
    tck, u = interpolate.splprep([x, y, z])
    print 'u:', u
    return tck, u


if __name__ == '__main__':
    load_plane()

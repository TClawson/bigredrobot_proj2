#!/usr/bin/env python

import rospy

import numpy as np
import helper_functions as hf
from baxter_pykdl import baxter_kinematics
from scipy import interpolate

from std_msgs.msg import (
    UInt16,
    String
)

import baxter_interface

from baxter_interface import CHECK_VERSION

class LineFollower(object):
    '''
    A class designed to follow a line in the workspace
    '''
    # BAD STUFF BAD STUFF BAD STUFF BAD STUFF BAD STUFF BAD STUFF BAD STUFF
    # *********************************************************************
    # Loop is set to just go for 3 seconds instead of using criteria
    # Move_speed is set to 0
    # KP is set to 0
    # *********************************************************************
    # BAD STUFF BAD STUFF BAD STUFF BAD STUFF BAD STUFF BAD STUFF BAD STUFF

    # KP = 0.8                # Line following proportional gain
    KP = 0
    MOVE_SPEED = 0          
    #    MOVE_SPEED = 0.07       # Velocity used when following the line
    K0 = -10                  # Gain for the secondary objective function
    DELTA = 0.01            # Step size for partial derivative calculation

    # Used for debugging - quickly save/load goals and a plane
    SAVE_PLANE = False
    LOAD_PLANE = not SAVE_PLANE
    SAVE_GOAL = True
    LOAD_GOAL = not SAVE_GOAL
   

    def __init__(self):
        rospy.logwarn("Initializing LineFollower")
        self.rate_publisher = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._left_joint_names = self._left_arm.joint_names()
        self._left_kin = baxter_kinematics('left')
        rospy.sleep(2)     

        # control parameters
        self.pub_rate = 500.0  # Hz

        rospy.loginfo("Getting robot state... ")
        self.interface = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self.interface.state().enabled
        rospy.loginfo("Enabling robot... ")
        self.interface.enable()
        
        self.lower_limits, self.upper_limits = hf.get_limits()

        # set joint state publishing to 500Hz
        self.rate_publisher.publish(self.pub_rate)

    def _reset_control_modes(self):
        '''
        Resets the joint publish rate and stops writing commands to Baxter
        '''
        rate = rospy.Rate(self.pub_rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self.rate_publisher.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True
    
    def set_neutral(self):
        '''
        Sets both arms back into a neutral pose.
        '''
        rospy.loginfo("Moving to neutral pose...")
        self._left_arm.move_to_neutral()

    def get_gripper_coords(self):
        pos = self._left_arm.endpoint_pose().popitem()[1]
        return np.matrix([pos.x, pos.y, pos.z]).T

    def clean_shutdown(self):
        rospy.loginfo("\nCrashing stuff...")
        self._reset_control_modes()
        return True

    def follow_line(self, p1, p2, v0):
        '''
        Deprecated - use follow_line_p_control instead
        Follows a straight line in the workspace
        '''
        rate = rospy.Rate(self.pub_rate)

        p12 = p2 - p1
        max_dist = np.linalg.norm(p12)
        v12 = p12/np.linalg.norm(p12)*v0
        squiggle = np.matrix([v12[0], v12[1], v12[2], 0, 0, 0]).T

        rospy.loginfo("Moving. Press Ctrl-C to stop...")
        while (self.dist_from_point(p1) < max_dist) and not rospy.is_shutdown():
            self.rate_publisher.publish(self.pub_rate)
            self.command_velocity(squiggle)
            rate.sleep()

    def follow_line_p_control(self, p1, p2, v0, kp):
        '''
        Follows a straight line in the workspace using proportional control
        '''
        rate = rospy.Rate(self.pub_rate)
        t0 = rospy.Time.now()
        t = 0
        p1 = p1.squeeze()
        p2 = p2.squeeze()
        p12 = p2 - p1 # relative position of p1 wrt p2
        max_dist = np.linalg.norm(p12)
        max_dist_actual = self.dist_from_point(p2)
        v12 = p12/np.linalg.norm(p12)*v0
        rospy.loginfo("Moving. Press Ctrl-C to stop...")
        # while (self.dist_from_point(p1) < max_dist_actual) and not rospy.is_shutdown():
        while (t < 3):
            self.rate_publisher.publish(self.pub_rate)
            t = (rospy.Time.now() - t0).to_sec()
            p_estimate = p1 + t*v12
            p_actual = self.get_gripper_coords().squeeze()
            error = p_estimate - p_actual
            v_correct = kp*error
            v_command = v12 + v_correct
            squiggle = np.concatenate((np.matrix(v_command).T, np.matrix([0,0,0]).T))
            self.command_velocity(squiggle)
            rate.sleep()
        rospy.loginfo("Done moving this line")
        self.command_velocity(np.matrix([0,0,0,0,0,0]).T);        

    def command_velocity(self, squiggle):
        '''
        Commands joint velocities using jacobian magicks
        '''
        J = self._left_kin.jacobian()
        Jinv = hf.rpinv(J)
        q_dot = Jinv*squiggle + (np.identity(7) - (Jinv*J))*self.get_b(self.K0, self.DELTA) 
        cmd = {joint: q_dot[i, 0] for i, joint in enumerate(self._left_joint_names)}
        self._left_arm.set_joint_velocities(cmd)
   
    def dist_from_point(self, p):
        '''
        Gets the distance of the gripper from some point p.
        'p' is a numpy column vector
        '''
        gripper = self.get_gripper_coords()
        r = gripper.squeeze() - p.squeeze()
        return np.linalg.norm(r)

    def get_w_joint_limits(self, joint_angles, lower_limits, upper_limits):
        '''
        upper_limits and lower_limits are dicts, so is joint_angles
        '''
        n = len(hf.frame_dict)
        w = 0
        for joint in hf.frame_dict:
            q_bar = (lower_limits[joint] + upper_limits[joint])/2
            #NOTE: Changed from w - ... to w + ... : might be wrong
            w = w + ((joint_angles[joint] - q_bar)/(upper_limits[joint] - lower_limits[joint]))**2
        return w

    def get_z_joint_limits(self, joint_angles, lower_limits, upper_limits):
        '''
        upper_limits and lower_limits are dicts, so is joint_angles
        '''
        n = len(hf.frame_dict)
        z = []
        for joint in hf.frame_dict:
            q_bar = (lower_limits[joint] + upper_limits[joint])/2
            #NOTE: Changed from w - ... to w + ... : might be wrong
            z.append((joint_angles[joint] - q_bar)/(upper_limits[joint] - lower_limits[joint])**2)
        return np.matrix(z).T

    def get_partial_w_q_joint_limits(self, joint_angles, delta):
        '''
        For the secondary objective function, get the partial of w wrt q
        '''
        # All inputs are dicts
        n = len(joint_angles)
        partial = []
        
        for joint in hf.frame_dict:
            delta_angles = joint_angles.copy()
            delta_angles[joint] = delta_angles[joint] + delta
            fq = self.get_w_joint_limits(joint_angles, self.lower_limits, self.upper_limits)
            fqdelta = self.get_w_joint_limits(delta_angles, self.lower_limits, self.upper_limits)
            partial.append( (fqdelta - fq) / delta) 
        return np.matrix(partial).T # np array in framedict order

    def get_b(self, k, delta):
        '''
        Secondary objective function, designed to avoid joint limits
        '''
        joint_angles = self._left_arm.joint_angles()
        # This is other stuff (thanks Google!)
        return k * self.get_z_joint_limits(joint_angles, self.lower_limits, self.upper_limits)
        # This is from class
        # return k * self.get_partial_w_q_joint_limits(joint_angles,delta)
     
    def follow_spline(self, tck, u, u_step=0.1):
        '''
        Follows a spline in the workspace
        '''
        u_min = min(u)
        u_max = max(u)
        new_u = np.arange(u_min, u_max+u_step, u_step)
        out = interpolate.splev(new_u, tck)
        x = out[0]
        y = out[1]
        z = out[2]
        for i in range(len(new_u)-1):
            p1 = np.array([x[i], y[i], z[i]])
            p2 = np.array([x[i+1], y[i+1], z[i+1]])
            self.follow_line_p_control(p1, p2, self.MOVE_SPEED, self.KP)

def main():
    rospy.loginfo("Initializing node... ")
    rospy.init_node("velocity_follower")
    line_follower = LineFollower()
    rospy.on_shutdown(line_follower.clean_shutdown)

    #Define a plane to write on
    if line_follower.LOAD_PLANE:
        rospy.loginfo("Loading plane from file..")
        point, normal = hf.load_plane()
        rospy.loginfo("Loaded")
    else:
        raw_input("Define the first point on the plane")
        plane1 = line_follower.get_gripper_coords()
        raw_input("Define the second point on the plane")
        plane2 = line_follower.get_gripper_coords()
        raw_input("Define the third point on the plane")
        plane3 = line_follower.get_gripper_coords()
        point, normal = hf.get_plane(plane1, plane2, plane3)

    if line_follower.SAVE_PLANE:
        hf.save_plane(point, normal)
    
    #Wait for command
    while not rospy.is_shutdown():
        spline_time = raw_input("Spline time? y/n")
        if spline_time == 'y':
            print "Spline mode entered"
            raw_input("Press enter to set goal")
            p4 = line_follower.get_gripper_coords()
            rospy.loginfo(p4)
            raw_input("Press enter to set midpoint1")
            p3 = line_follower.get_gripper_coords()
            rospy.loginfo(p3)
            raw_input("Press enter to set midpoint2")
            p2 = line_follower.get_gripper_coords()
            rospy.loginfo(p2)
            raw_input("Press enter to set start")
            p1 = line_follower.get_gripper_coords()
            rospy.loginfo(p1)

            p1 = hf.project_point(point, normal, p1)
            p2 = hf.project_point(point, normal, p2)
            p3 = hf.project_point(point, normal, p3)
            p4 = hf.project_point(point, normal, p4)

            tck, u = hf.get_spline(p1, p2, p3, p4)
            line_follower.follow_spline(tck, u)
    
        else:
            print "Line mode entered"
            if line_follower.LOAD_GOAL:
                raw_input("Press enter to load goal from file...")
                p1, p2 = hf.load_goal()
            else:
                raw_input("Press enter to set goal")
                p2 = line_follower.get_gripper_coords()
                rospy.loginfo(p2)
                raw_input("Press enter to set start")
                p1 = line_follower.get_gripper_coords()
                rospy.loginfo(p1)
                hf.save_goal(p1, p2)
            rospy.loginfo('Following...')
       
            p1 = hf.project_point(point, normal, p1)
            p2 = hf.project_point(point, normal, p2)

            line_follower.follow_line_p_control(p1, p2, line_follower.MOVE_SPEED, line_follower.KP)    
    rospy.loginfo("Done.")

if __name__ == '__main__':
    main()

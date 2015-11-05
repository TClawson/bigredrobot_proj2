#!/usr/bin/env python

import math
import rospy
import rospkg
import tf
import rrt
import numpy as np
import HelperFunctions as hf
from baxter_pykdl import baxter_kinematics

from std_msgs.msg import (
    UInt16,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class LineFollower(object):

    KP = 0.8
    CONFIG_KP = 0.1
    DIST_THRESH = 0.01
    MOVE_SPEED = 0.07
    CONFIG_V0 = 0.4
    SAVE_PLANE = False
    LOAD_PLANE = not SAVE_PLANE
    SAVE_GOAL = True
    LOAD_GOAL = not SAVE_GOAL
    K0 = 0
    DELTA = 0.01

    def __init__(self):
        rospy.loginfo("Initializing LineFollower")
        self.rate_publisher = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._left_joint_names = self._left_arm.joint_names()
        self._left_kin = baxter_kinematics('left')
        self._listener =  tf.TransformListener()
        rospy.sleep(2)
        self._listener.waitForTransform('base', 'base', rospy.Time(), rospy.Duration(4)) # wait for transforms to publish      

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
        rate = rospy.Rate(self.pub_rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self.rate_publisher.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        rospy.loginfo("Moving to neutral pose...")
        self._left_arm.move_to_neutral()

    def get_gripper_coords(self):
        pos = self._left_arm.endpoint_pose().popitem()[1]
        return np.matrix([pos.x, pos.y, pos.z]).T

    def get_current_config(self):
        d = self._left_arm.joint_angles()
        cur_config = []
        for joint in self._left_joint_names:
            cur_config.append(d[joint])

        return np.array(cur_config)

    def clean_shutdown(self):
        rospy.loginfo("\nCrashing stuff...")
        #return to normal
        self._reset_control_modes()
        #self.set_neutral()
        #if not self._init_state:
        #    rospy.loginfo("Disabling robot...")
        #    self.interface.disable()
        return True

    def follow_line(self, p1, p2, v0):
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
        rate = rospy.Rate(self.pub_rate)
        t0 = rospy.Time.now()

        p12 = p2 - p1 # relative position of p1 wrt p2
        max_dist = np.linalg.norm(p12)
        v12 = p12/np.linalg.norm(p12)*v0

        rospy.loginfo("Moving. Press Ctrl-C to stop...")
        while (self.dist_from_point(p1) < max_dist) and not rospy.is_shutdown():
            self.rate_publisher.publish(self.pub_rate)
            t = (rospy.Time.now() - t0).to_sec()
            p_estimate = p1 + t*v12
            p_actual = self.get_gripper_coords()
            error = p_estimate - p_actual
            v_correct = kp*error
            v_command = v12 + v_correct
            squiggle = np.concatenate((v_command, np.matrix([0,0,0]).T))
            self.command_velocity(squiggle)
            rate.sleep()

        self.command_velocity(np.matrix([0,0,0,0,0,0]).T);        

    def follow_line_c_space(self, q_init, q_goal, v0, kp):
        t0 = rospy.Time.now()
        rate = rospy.Rate(self.pub_rate)

        target_size = 0.05

        relpos = q_goal - q_init
        max_dist_config = np.linalg.norm(relpos)
        movedir = relpos/max_dist_config
        vel = movedir*v0

        q_init_actual = self.get_current_config()
        max_dist_actual = np.linalg.norm(q_goal - q_init_actual)

        # TODO With too fine a discretization, this still overshoots the goal somehow
        while(self.dist_from_config(q_init_actual) < max_dist_actual) and not rospy.is_shutdown():
            print 'dist_from_config(q_init_actual):'
            print self.dist_from_config(q_init_actual)                         
            self.rate_publisher.publish(self.pub_rate)
            #t = (rospy.Time.now() - t0).to_sec()
            #q_estimate = q_init + t*vel
            #if np.linalg.norm(q_estimate - q_init) > max_dist_config:
            #    q_estimate = q_goal
            q_actual = self.get_current_config()
            #error = q_estimate - q_actual
            error = q_goal - q_actual
            v_correct = kp*error
            if np.linalg.norm(v_correct) > v0: # saturate controller at v0
                v_correct = v0*v_correct/np.linalg.norm(v_correct) # rescale to velocity v0
            #v_command = vel + v_correct
            v_command = v_correct            
            self.command_config_velocity(v_command)
            rate.sleep()

    def follow_path_c_space(self, path, v0, kp): 
        rospy.loginfo("Moving. Press Ctrl-C to stop...")
        for i in range(path.shape[0] - 1):

            q_init = path[i,:]
            q_goal = path[i+1,:]
            self.follow_line_c_space(q_init, q_goal, v0, kp)
        #Stop moving Baxter
        self.command_config_velocity(np.matrix([0,0,0,0,0,0,0]).T)
            
    def command_velocity(self, squiggle):
        J = self._left_kin.jacobian()
        Jinv = hf.rpinv(J)
        q_dot = Jinv*squiggle + (np.identity(7) - (Jinv*J))*self.get_b(self.K0, self.DELTA) 
        cmd = {joint: q_dot[i, 0] for i, joint in enumerate(self._left_joint_names)}
        self._left_arm.set_joint_velocities(cmd)

    def command_config_velocity(self, q_dot):
        cmd = {joint: q_dot[i] for i, joint in enumerate(self._left_joint_names)}
        self._left_arm.set_joint_velocities(cmd)

    def dist_from_config(self, q):
        q_cur = np.array(self.get_current_config())
        return np.linalg.norm(q_cur - q)

    def dist_from_point(self, p):
        '''
        Gets the distance of the gripper from some point p.
        'p' is a numpy column vector
        '''
        gripper = self.get_gripper_coords()
        r = gripper - p
        return np.linalg.norm(r)

    def get_w(self, joint_angles, lower_limits, upper_limits):
        '''
        upper_limits and lower_limits are dicts, so is joint_angles
        '''
        n = len(hf.frame_dict)
        w = 0
        for joint in hf.frame_dict:
            q_bar = (lower_limits[joint] + upper_limits[joint])/2
            w = w - 1.0/(2.0*n)*((joint_angles[joint] - q_bar)/(upper_limits[joint] - lower_limits[joint]))**2
        return w

    def get_partial_w_q(self, joint_angles, delta):
        # dicts (everything)
        n = len(joint_angles)
        partial = []
        
        for joint in hf.frame_dict:
            delta_angles = joint_angles.copy()
            delta_angles[joint] = delta_angles[joint] + delta
            fq = self.get_w(joint_angles, self.lower_limits, self.upper_limits)
            fqdelta = self.get_w(delta_angles, self.lower_limits, self.upper_limits)
            partial.append( (fqdelta - fq) / delta)
        return np.matrix(partial).T # np array in framedict order

    def get_b(self, k, delta):
        '''
        Secondary objective function
        '''
        joint_angles = self._left_arm.joint_angles()
        
        return k * self.get_partial_w_q(joint_angles,delta)

        '''
        dw = []

        n = len(joint_infos)
        for joint_info in joint_infos:
            q_bar = (joint_info[1] + joint_info[2])/2
            
            dw.append(-1/(2n)*((joint_info[0] + delta - q_bar)/(joint_info[2] - joint_info[1]))**2)

        return k*np.matrix(w).T
    '''
def main():

    rospy.loginfo("Initializing node... ")
    rospy.init_node("velocity_follower")
    line_follower = LineFollower()
    rospy.on_shutdown(line_follower.clean_shutdown)

    #Define plane
    if line_follower.LOAD_PLANE:
        rospy.loginfo("Loading plane from file..")
        point, normal = hf.load_plane()
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
        
    while not rospy.is_shutdown():
        raw_input("Press enter to set goal")
        q2 = line_follower.get_current_config()
        raw_input("Press enter to set start")
        q1 = line_follower.get_current_config()
        path = rrt.plan_rrt_connect(q1, q2, epsilon=0.01)
        # path = rrt.smooth_path(path)
        # DONT USE WORKSPACE VELOCITY AND KP HERE
        line_follower.follow_path_c_space(path, line_follower.CONFIG_V0, line_follower.CONFIG_KP)
        #line_follower.follow_line_c_space(q1, q2, line_follower.CONFIG_V0, line_follower.CONFIG_KP)
    '''
        #Wait for command
        while not rospy.is_shutdown():
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

    #        line_follower.follow_line(p1, p2, line_follower.MOVE_SPEED)     #Almost certainly possibly meters/second
            line_follower.follow_line_p_control(p1, p2, line_follower.MOVE_SPEED, line_follower.KP)     #Almost certainly possibly meters/second
    '''
    rospy.loginfo("Done.")

if __name__ == '__main__':
    main()

#!/usr/bin/env python

import math
import rospy
import rospkg
import tf
import rrt
import numpy as np
import helper_functions as hf
from baxter_pykdl import baxter_kinematics

from std_msgs.msg import (
    UInt16,
    String,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class LineFollower(object):


    CONFIG_KP = 0.1
    DIST_THRESH = 0.01
    CONFIG_V0 = 0.4
   

    def __init__(self):
        rospy.logwarn("Initializing ")
        self.rate_publisher = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._left_joint_names = self._left_arm.joint_names()
        self._left_kin = baxter_kinematics('left')
        rospy.sleep(2)
        rospy.loginfo("Stuff")     

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

        rospy.wait_for_service('check_collision')

        rospy.Subscriber("baxter_input", String, self.input_callback)
        self.input_received = False

    def _reset_control_modes(self):
        rate = rospy.Rate(self.pub_rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self.rate_publisher.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True
    
    def input_callback(self, data):
        self.input_received = True

    def wait_for_input(self):
        while not rospy.is_shutdown() and not self.input_received:
            pass
        self.input_received = False
            

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
            self.rate_publisher.publish(self.pub_rate)
            t = (rospy.Time.now() - t0).to_sec()
            q_estimate = q_init + t*vel
            if np.linalg.norm(q_estimate - q_init) > max_dist_config:
                q_estimate = q_goal
            q_actual = self.get_current_config()
            error = q_estimate - q_actual
            v_correct = kp*error      
            v_command = vel + v_correct
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

   
def main():
#  <include file="$(find collision_checker)/launch/collision_server.launch"/>
    rospy.logwarn("Initializing node... ")
    rospy.init_node("velocity_follower")
    line_follower = LineFollower()
    rospy.on_shutdown(line_follower.clean_shutdown)

    while not rospy.is_shutdown():
        rospy.sleep(3)
        #line_follower.set_neutral()
        raw_input("Press enter to set goal")
        q2 = line_follower.get_current_config()
        raw_input("Press enter to set start")
        q1 = line_follower.get_current_config()

        #simulation stuff
        #q1 = q2 + np.array([-0.3,0,0,0,0,0,0])

        rospy.logwarn("Planning path....")
        path = rrt.plan_rrt_connect(q1, q2, epsilon=0.05)#rrt.plan_rrt_connect(q1, q2, epsilon=0.01)
        rospy.logwarn("Smoothing path....")
        path = rrt.smooth_path(path, max_iter = 80)
        # DONT USE WORKSPACE VELOCITY AND KP HERE
        rospy.logwarn("Following path....")
        line_follower.follow_path_c_space(path, line_follower.CONFIG_V0, line_follower.CONFIG_KP)
        #line_follower.follow_line_c_space(q1, q2, line_follower.CONFIG_V0, line_follower.CONFIG_KP)
   
    rospy.loginfo("Done.")

if __name__ == '__main__':
    main()

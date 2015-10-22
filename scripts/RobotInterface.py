#!/usr/bin/env python

import math
import rospy
import tf
import numpy as np
import HelperFunctions as hf

from std_msgs.msg import (
    UInt16,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class LineFollower(object):

    DIST_THRESH = 0.1

    def __init__(self):
        rospy.loginfo("Initializing LineFollower")
        self.rate_publisher = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._left_joint_names = self._left_arm.joint_names()
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
        return np.array([pos.x, pos.y, pos.z])

    def clean_shutdown(self):
        rospy.loginfo("\nCrashing stuff...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            rospy.loginfo("Disabling robot...")
            self.interface.disable()
        return True

    def follow_line(self, p1, p2, v0):
        rate = rospy.Rate(self.pub_rate)

        p12 = p2 - p1
        
        v12 = p12/np.linalg.norm(p12)*v0
        arm = 0
        squiggle = np.array([v12[0], v12[1], v12[2], 0, 0, 0])
        squiggle.shape = (6,1) # force to be col vector
        rospy.loginfo("Moving. Press Ctrl-C to stop...")
        while (self.dist_from_point(p2) > self.DIST_THRESH) and not rospy.is_shutdown():
            self.rate_publisher.publish(self.pub_rate)
            transforms = hf.get_transforms(self._listener, self._left_arm)
            j = hf.jacobian(transforms)
            q_dot = np.dot(hf.lpinv(j), squiggle)
            rospy.loginfo(q_dot)
            cmd = {joint: q_dot[i] for i, joint in enumerate(self._left_joint_names)}
            self._left_arm.set_joint_velocities(cmd)

            rate.sleep()

    def follow_line_p_control(self, p1, p2, v0, kp):
        rate = rospy.Rate(self.pub_rate)
        t0 = rospy.Time.now()

        p12 = p2 - p1
        v12 = p12/np.linalg.norm(r12)*v0
        arm = 0
        while (self.dist_from_point(p2) > self.DIST_THRESH) and not rospy.is_shutdown():
            self.rate_publisher.publish(self.pub_rate)
            t = rospy.Time.now() - t0
            p_estimate = p1 + t*v12
            p_actual = self.get_gripper_coords()
            error = p_actual - p_estimate
            v_correct = kp*error
            v_command = v12 + v_correct
            squiggle = concatenate(v_command.T, np.array[0,0,0])
            squiggle.shape = (6,1) # force to be col vector
            rospy.loginfo("Moving. Press Ctrl-C to stop...")
            hf.get_transforms(self._listener, self._left_arm)
            j = hf.jacobian(transforms)
            q_dot = hf.lpinv(j)*squiggle
            cmd = {[(joint, q_dot[i]) for i, joint in enumerate(self._left_joint_names)]}
            self._left_arm.set_joint_velocities(cmd)

            rate.sleep()        

    def dist_from_point(self, p):
        '''
        Gets the distance of the gripper from some point p.
        'p' is a numpy array
        '''
        gripper = self.get_gripper_coords()
        r = np.array(gripper) - p
        return np.sqrt(r[0]**2 + r[1]**2 + r[2]**2)

def main():

    rospy.loginfo("Initializing node... ")
    rospy.init_node("black_sheep_wobbler")
    line_follower = LineFollower()
    rospy.on_shutdown(line_follower.clean_shutdown)
    p1 = line_follower.get_gripper_coords();
    p2 = p1 + np.array([0.2,0,0])     #Meters? Probably...
    line_follower.follow_line(p1, p2, 0.02)  #Almost certainly possibly meters/second

    rospy.loginfo("Done.")

if __name__ == '__main__':
    main()

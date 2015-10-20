#!/usr/bin/env python

import math
import rospy

import numpy as np
import HelperFunctions as hf

from std_msgs.msg import (
    UInt16,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class LineFollower(object):

    def __init__(self):
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._left_joint_names = self._left_arm.joint_names()

        # control parameters
        self._rate = 500.0  # Hz

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self._pub_rate.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()

    def get_gripper_coords(self):
        pose = self.left_limb.endpoint_pose().popitem()[1]
        return np.array([pos.x, pos.y, pos.z])

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def follow_line(self, p1, p2, v0):
        rate = rospy.Rate(self._rate)

        p12 = p2 - p1
        v12 = p12/np.linalg.norm(r12)*v0
        arm = 0
        squiggle = np.array([v12(0), v12(1), v12(2), 0, 0, 0]).T
        while (self.get_gripper_coords() != p2) and not rospy.is_shutdown():
            self._pub_rate.publish(self._rate)
            print("Moving. Press Ctrl-C to stop...")
            hf.get_transforms(self._left_arm)
            j = hf.jacobian(transforms)
            q_dot = lpinv(j)*squiggle
            cmd = {[(joint, q_dot[i]) for i, joint in enumerate(self._left_joint_names)]}
            self._left_arm.set_joint_velocities(cmd)

            rate.sleep()

    def follow_line_p_control(self, p1, p2, v0, kp):
        rate = rospy.Rate(self._rate)
        t0 = rospy.Time.now()

        p12 = p2 - p1
        v12 = p12/np.linalg.norm(r12)*v0
        arm = 0
        while (self.get_gripper_coords() != p2) and not rospy.is_shutdown():
            self._pub_rate.publish(self._rate)
            t = rospy.Time.now() - t0
            p_estimate = p1 + t*v12
            p_actual = self.get_gripper_coords()
            error = p_actual - p_estimate
            v_correct = kp*error
            v_command = v12 + v_correct
            zeros = np.array([0, 0, 0]).T
            squiggle = concatenate(v_command.T, zeros)
            print("Moving. Press Ctrl-C to stop...")
            hf.get_transforms(self._left_arm)
            j = hf.jacobian(transforms)
            q_dot = lpinv(j)*squiggle
            cmd = {[(joint, q_dot[i]) for i, joint in enumerate(self._left_joint_names)]}
            self._left_arm.set_joint_velocities(cmd)

            rate.sleep()        

def main():

    print("Initializing node... ")
    rospy.init_node("black_sheep_wobbler")

    line_follower = LineFollower()
    rospy.on_shutdown(line_follower.clean_shutdown)
    p1 = line_follower.get_gripper_coords();
    p2 = p1;
    p2[0] = p2[0] + 1;

    line_follower.follow_line(p1, p2, 0.2)

    print("Done.")

if __name__ == '__main__':
    main()

#!/usr/bin/env python

import math
import rospy
import tf
import numpy as np
import HelperFunctions as hf
from baxter_pykdl import baxter_kinematics

from std_msgs.msg import (
    UInt16,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class LineFollower(object):

    DIST_THRESH = 0.01
    MOVE_SPEED = 0.2

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

        p12 = p2 - p1
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

    def command_velocity(self, squiggle):
        J = self._left_kin.jacobian()
        q_dot = hf.rpinv(J)*squiggle
        cmd = {joint: q_dot[i, 0] for i, joint in enumerate(self._left_joint_names)}
        self._left_arm.set_joint_velocities(cmd)

    def dist_from_point(self, p):
        '''
        Gets the distance of the gripper from some point p.
        'p' is a numpy column vector
        '''
        gripper = self.get_gripper_coords()
        r = gripper - p
        return np.linalg.norm(r)

def main():

    rospy.loginfo("Initializing node... ")
    rospy.init_node("velocity_follower")
    line_follower = LineFollower()
    rospy.on_shutdown(line_follower.clean_shutdown)
    
    #Define plane
    raw_input("Define the first point on the plane")
    plane1 = line_follower.get_gripper_coords()
    raw_input("Define the second point on the plane")
    plane2 = line_follower.get_gripper_coords()
    raw_input("Define the third point on the plane")
    plane3 = line_follower.get_gripper_coords()
    point, normal = hf.get_plane(plane1, plane2, plane3)

    #Wait for command
    while not rospy.is_shutdown():
        raw_input("Press enter to set goal")
        p2 = line_follower.get_gripper_coords()
        rospy.loginfo(p2)
        raw_input("Press enter to set start")
        p1 = line_follower.get_gripper_coords()
        rospy.loginfo(p1)
        rospy.loginfo('Following...')
       
        p1 = hf.project_point(point, normal, p1)
        p2 = hf.project_point(point, normal, p2)

#        line_follower.follow_line(p1, p2, line_follower.MOVE_SPEED)     #Almost certainly possibly meters/second
        line_follower.follow_line_p_control(p1, p2, line_follower.MOVE_SPEED, 0.7)     #Almost certainly possibly meters/second

    rospy.loginfo("Done.")

if __name__ == '__main__':
    main()

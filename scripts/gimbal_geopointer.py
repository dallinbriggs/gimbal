#!/usr/bin/env python
from __future__ import print_function
import argparse
import numpy as np
import rospy
import math
import tf

from geometry_msgs.msg import Vector3, Vector3Stamped
from rosflight_msgs.msg import GPS
from rosplane_msgs.msg import State, Current_Path, Controller_Commands
from nav_msgs.msg import Odometry
# from inertial_sense.msg import GPS
from std_msgs.msg import UInt16


class Geopointer(object):

    def __init__(self):
        """
        Points a gimbal at a given coordinate in the inertial frame
        """

        # initialize the target to be the origin
        self.target_pos = np.array([0., 0., 0.])
        self.pos = np.array([0., 0., 0.])
        self.phi = 0
        self.theta = 0
        self.psi = 0
        # initialize the line of sight vector to point from the position
        # to the origin
        self.los = np.array([0., 0., 0.])
        self.psiLast = 0.
        self.psiWrap = 0.

        self.gimbal_az = 0.0
        self.gimbal_el = 0.0

        # rospy.Subscriber("state", State, self.mav_pos_callback)
        # rospy.Subscriber("state", State, self.mav_state_callback)
        rospy.Subscriber("ins/inertial", Odometry, self.mav_state_callback)
        # rospy.Subscriber("target_pos", Vector3, self.target_callback)

        self.gimbal_pub = rospy.Publisher("gimbal/control", Vector3Stamped, queue_size=1)

    def target_callback(self, msg):
        # get the current target position in the inertial frame
        # self.target_pos = np.array([msg.latitude, msg.longitude, -msg.altitude])
        self.target_pos = np.array([msg.x, msg.y, msg.z])

    def mav_state_callback(self, msg):

        self.phi = msg.phi
        self.theta = msg.theta
        self.psi = msg.psi

        self.pos = np.array(msg.position)

    def compute_gimbal_control(self):
        # find the vector pointing from the mav to the target
        # self.los = self.target_pos - self.pos
        self.los = np.array([0., 0., 0.]) - self.pos
        # rotate los vector into body frame
        cPhi = math.cos(self.phi)
        sPhi = math.sin(self.phi)
        cTheta = math.cos(self.theta)
        sTheta = math.sin(self.theta)
        cPsi = math.cos(self.psi)
        sPsi = math.sin(self.psi)
        R_v_to_body = np.array([[                 cTheta*cPsi,                   cTheta*sPsi,     -1*sTheta],
                                [sPhi*sTheta*cPsi - cPhi*sPsi,  sPhi*sTheta*sPsi + cPhi*cPsi,   sPhi*cTheta],
                                [cPhi*sTheta*cPsi + sPhi*sPsi,  cPhi*sTheta*sPsi - sPhi*cPsi,   cPhi*cTheta]])
        self.los = np.dot(R_v_to_body, self.los)

        # normalize the los vector
        norm = np.linalg.norm(self.los)
        if norm > 0:
            self.los = self.los / norm

        self.gimbal_az = math.atan2(self.los[1], self.los[0])
        self.gimbal_el = -1*math.asin(self.los[2])

        # publish the result to the gimbal control
        vec3Stamped = Vector3Stamped()
        vec3Stamped.header.stamp = rospy.Time.now()
        vec = Vector3(0, self.gimbal_el, self.gimbal_az)
        vec3Stamped.vector = vec
        self.gimbal_pub.publish(vec3Stamped)


    def debug_print(self, string, value, label):
        want_to_print = (label == "none"  # Default to not print
                        )

        # Only print the debug messages with the specified label(s)
        if want_to_print:
            print(string, end='') # Print with no newline
            print(value)

    def clean_shutdown(self):
        print("\nExiting pointer...")
        return True

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.compute_gimbal_control()
            rate.sleep()

def main():
    """fcu_sim compatible gimbal geopointer

    Commands gimbal azimuth
    """

    print("Initializing node... ")
    rospy.init_node("gimbal_geopointer", log_level=rospy.DEBUG)

    geopointer = Geopointer()
    rospy.on_shutdown(geopointer.clean_shutdown)
    geopointer.run()

    print("Done.")

if __name__ == '__main__':
    main()

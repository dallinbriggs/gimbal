#!/usr/bin/env python
from __future__ import print_function
import argparse
import numpy as np
import rospy
import math
import tf
# import utilities

from geometry_msgs.msg import Vector3, Vector3Stamped
from rosflight_msgs.msg import State, GPS
from ros_plane.msg import Current_Path, Controller_Commands
from nav_msgs.msg import Odometry
# from inertial_sense.msg import GPS
# from rosflight_msgs.msg import Attitude
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
        rospy.Subscriber("state", State, self.mav_state_callback)
        rospy.Subscriber("target_pos", Vector3, self.target_callback)

        # self.az_pub = rospy.Publisher("gimbal_yaw", UInt16, queue_size=1)
        # self.el_pub = rospy.Publisher("gimbal_pitch", UInt16, queue_size=1)
        self.gimbal_pub = rospy.Publisher("gimbal/control", Vector3Stamped, queue_size=1)

    def target_callback(self, msg):
        # get the current target position in the inertial frame
        # self.target_pos = np.array([msg.latitude, msg.longitude, -msg.altitude])
        self.target_pos = np.array([msg.x, msg.y, msg.z])

    def mav_state_callback(self, msg):
        # get the current mav attitude in the inertial frame
        # quaternion = (
        #     msg.attitude.x,
        #     msg.attitude.y,
        #     msg.attitude.z,
        #     msg.attitude.w)
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # self.phi = euler[0]
        # self.theta = euler[1]
        # self.psi = euler[2]

        self.phi = msg.phi
        self.theta = msg.theta
        self.psi = msg.psi

        self.pos = np.array(msg.position)

    def compute_gimbal_control(self):
        # find the vector pointing from the mav to the target
        self.los = self.target_pos - self.pos

        # rotate los vector into body frame
        cPhi = math.cos(self.phi)
        sPhi = math.sin(self.phi)
        cTheta = math.cos(self.theta)
        sTheta = math.sin(self.theta)
        cPsi = math.cos(self.psi)
        sPsi = math.sin(self.psi)
        R_body_to_v = np.array([[cTheta*cPsi,  sPhi*sTheta*cPsi - cPhi*sPsi,  cPhi*sTheta*cPsi + sPhi*sPsi],
                                [cTheta*sPsi,  sPhi*sTheta*sPsi + cPhi*cPsi,  cPhi*sTheta*sPsi - sPhi*cPsi],
                                [  -1*sTheta,                   sPhi*cTheta,                   cPhi*cTheta]])
        R_v_to_body = np.asarray(np.matrix(R_body_to_v).transpose())
        self.los = np.dot(R_v_to_body, self.los)

        # normalize the gimbal vector
        norm = np.linalg.norm(self.los)
        if norm > 0:
            self.los = self.los / norm

        down = np.array([0., 0., 1.])

        # find a vector that points 90 degrees to the right of the object, along
        # the horizon. This will ensure the gimbal stays level when watching the
        # target
        right = np.cross(down, self.los)
        # normalize it
        norm = np.linalg.norm(right)
        if norm > 0:
            right = right / norm
        # recalculate down to be orthogonal to los and right
        down = np.cross(self.los, right)

        # get the euler angles that the gimbal needs to point along the los Vector
        psi = math.atan2(-right[0], right[1])
        psi -= math.pi/2.0

        # compensate for wraparound
        if   (psi - self.psiLast) > 1.9*math.pi:
            self.psiWrap -= 2*math.pi
        elif (psi - self.psiLast) < -1.9*math.pi:
            self.psiWrap += 2*math.pi
        self.psiLast = psi

        phi = math.asin(right[2])
        theta = math.atan2(-self.los[2], down[2])

        self.gimbal_az = psi
        self.gimbal_el = theta

        # publish the result to the gimbal control
        vec3Stamped = Vector3Stamped()
        vec3Stamped.header.stamp = rospy.Time.now()
        vec = Vector3(phi, theta, psi + self.psiWrap)
        vec3Stamped.vector = vec
        self.gimbal_pub.publish(vec3Stamped)

    #     self.publish_commands()
    #
    # def publish_commands(self):
    #     self.az_pub.publish(self.gimbal_az)
    #     self.el_pub.publish(self.gimbal_el)
    #     self.gimbal_pub.publish()

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

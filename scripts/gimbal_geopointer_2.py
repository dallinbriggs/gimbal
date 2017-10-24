#!/usr/bin/env python
import numpy as np
import rospy
import math
import utm

from geometry_msgs.msg import Vector3, Vector3Stamped, PoseStamped
from rosflight_msgs.msg import GPS
from rosplane_msgs.msg import Current_Path, Controller_Commands, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class Geopointer(object):

    def __init__(self):
        """
        Points a gimbal at a given GPS coordinate in the inertial frame
        """
        # initialize the target to be the origin for now
        self.target_gps_pos = np.array([0., 0., 0.])
        self.mav_gps_pos = np.array([0.,0.,0.])
        self.target_state_pos = np.array([0., 0., 0.])
        self.mav_state_pos = np.array([0.,0.,0.])
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

        self.use_gps_position = rospy.get_param("~use_gps_position", default=False)

        rospy.Subscriber("mav_gps", GPS, self.mav_gps_callback)
        rospy.Subscriber("target_gps", GPS, self.target_gps_callback)
        rospy.Subscriber("target_state", PoseStamped, self.target_state_callback)
        rospy.Subscriber("mav_state", State, self.mav_state_callback)

        self.az_pub = rospy.Publisher("gimbal_yaw", Float32, queue_size=1)
        self.el_pub = rospy.Publisher("gimbal_pitch", Float32, queue_size=1)
        self.gimbal_pub = rospy.Publisher("gimbal/control", Vector3Stamped, queue_size=1)

    def target_gps_callback(self, msg):
        # Convert gps coordinates to UTM [m] position
        [easting, northing, zone_number, zone_letter] = utm.from_latlon(msg.latitude, msg.longitude)
        self.target_gps_pos = np.array([northing, easting, -msg.altitude])

    def target_state_callback(self, msg):
        position = msg.pose.position
        self.target_state_pos = np.array([position.x, -position.y, -position.z]) # Convert from NWU to NED

    def mav_gps_callback(self, msg):
        # Convert gps coordinates to UTM [m] position
        [easting, northing, zone_number, zone_letter] = utm.from_latlon(msg.latitude, msg.longitude)
        self.mav_gps_pos = np.array([northing, easting, -msg.altitude])

    def mav_state_callback(self, msg):
        self.mav_state_pos = msg.position
        self.phi = msg.phi
        self.theta = msg.theta
        self.psi = msg.psi

        # self.compute_gimbal_control()

    def compute_gimbal_control(self):
        # find the vector pointing from the mav to the target
        if self.use_gps_position:
            self.los = self.target_gps_pos - self.mav_gps_pos
        else:
            self.los = self.target_state_pos - self.mav_state_pos


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

        self.publish_commands()

    def publish_commands(self):
        self.az_pub.publish(self.gimbal_az)
        self.el_pub.publish(self.gimbal_el)

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

    Commands gimbal azimuth and elevation
    """

    print("Initializing node... ")
    rospy.init_node("gimbal_geopointer", log_level=rospy.DEBUG)

    geopointer = Geopointer()
    rospy.on_shutdown(geopointer.clean_shutdown)
    geopointer.run()

    print("Done.")

if __name__ == '__main__':
    main()

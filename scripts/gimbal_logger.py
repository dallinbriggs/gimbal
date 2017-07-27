from __future__ import print_function
import argparse
import numpy as np
import rospy
import math
import tf

from geometry_msgs.msg import Vector3, Vector3Stamped
from rosflight_msgs.msg import GPS
from ros_plane.msg import State, Current_Path, Controller_Commands
from nav_msgs.msg import Odometry
# from inertial_sense.msg import GPS
# from rosflight_msgs.msg import Attitude
from std_msgs.msg import UInt16

# from gimbal.msg import Gimbal_log

class Gimbal_log(object):

    def __init__(self):

        #subscribe to the required topics
        rospy.Subscriber("state", State, self.mav_state_callback)
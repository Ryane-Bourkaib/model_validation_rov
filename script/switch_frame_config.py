#!/usr/bin/env python
import math
from os import kill
import string
import numpy as np
import rospy
import tf
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import ParamSet
from mavros_msgs.srv import ParamGet
from mavros_msgs.msg import ParamValue


from geometry_msgs.msg import Twist
import time
import sys
import argparse


#Frame configs inside Ap_Motos.h of pixwhak code
#    // Supported frame types
#   typedef enum {
#      SUB_FRAME_BLUEROV1,
#     SUB_FRAME_VECTORED,
#        SUB_FRAME_VECTORED_6DOF,
#        SUB_FRAME_VECTORED_6DOF_90DEG,
#        SUB_FRAME_SIMPLEROV_3,
#        SUB_FRAME_SIMPLEROV_4,
#        SUB_FRAME_SIMPLEROV_5,
#        SUB_FRAME_CUSTOM,
#    } sub_frame_t;






def get_frame_config():
    rospy.wait_for_service('mavros/param/get')
    try:
        paramService = rospy.ServiceProxy('mavros/param/get', ParamGet)
        getfeedback = paramService('FRAME_CONFIG')
        value = getfeedback.value 
        rospy.loginfo("get frame config Succeeded")
        print("bool:",getfeedback.success,", value of FRAME_CONFIG = ",value)
        return value;
    except (rospy.ServiceException), e:
        rospy.loginfo("Except get_frame_config")


def set_frame_config_custom_8_motors():
    rospy.wait_for_service('mavros/param/set')
    try:
        paramService = rospy.ServiceProxy('mavros/param/set', ParamSet)
        pv = ParamValue()
        pv.integer = 7
        pv.real = 0.0
        setfeedback = paramService("FRAME_CONFIG",pv) # 7 is custom Frame config (8 motors separately)
        value = setfeedback.value 
        rospy.loginfo("set frame config Succeeded")
        print("bool:",setfeedback.success,", value of FRAME_CONFIG = ",value, " ==> PLEASE REBOOT THE ROBOT TO TAKE INTO ACCOUNT THE NEW CONFIG")
    except (rospy.ServiceException), e:
        rospy.loginfo("Except set_frame_config_custom_8_motors")

def set_frame_config_normal_6DOF():
    rospy.wait_for_service('mavros/param/set')
    try:
        paramService = rospy.ServiceProxy('mavros/param/set', ParamSet)
        pv = ParamValue()
        pv.integer = 2
        pv.real = 0.0
        setfeedback = paramService('FRAME_CONFIG',pv) # 2 is 6DOF Frame config
        value = setfeedback.value 
        rospy.loginfo("set frame config Succeeded")
        print("bool:",setfeedback.success,", value of FRAME_CONFIG = ",value, " ==> PLEASE REBOOT THE ROBOT TO TAKE INTO ACCOUNT THE NEW CONFIG")
    except (rospy.ServiceException), e:
        rospy.loginfo("Except set_frame_config_normal_6DOF")

# if __name__ == '__main__':

#     #armDisarm(False)  # Not automatically disarmed at startup
#     rospy.init_node('switch_frame_config', anonymous=False)

#     config = get_frame_config()
# #   set_frame_config_custom_8_motors()
#     set_frame_config_normal_6DOF()

#     rospy.spin()


#!/usr/bin/env python3

import sys
import rospy
from ros_flir_spinnaker.srv import *

if __name__ == "__main__":
    rospy.wait_for_service('phm/phm_flir_spinnaker/execute')
    try:
        exefunc = rospy.ServiceProxy('phm/phm_flir_spinnaker/execute', ExecuteCommand)
        resp = exefunc('AutoFocus')
        print("Response: " + str(resp.state))
    except rospy.ServiceException as e:
        print(e)

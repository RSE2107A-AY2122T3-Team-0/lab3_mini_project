#!/usr/bin/env python

import rospy
from lab3_mini_project.srv import GetLimoStatus, GetLimoStatusResponse, GetLimoStatusRequest
from limo_base.msg import LimoStatus

def handle_get_status_req(req):
    response_msg = GetLimoStatusResponse()
    response_string = "Nothing here yet"

    if req.get_status == GetLimoStatusRequest.GET_STATUS_BATTERY_VOLTAGE:
        pass
    elif req.get_status == GetLimoStatusRequest.GET_STATUS_CONTROL_MODE:
        pass
    elif req.get_status == GetLimoStatusRequest.GET_STATUS_ERROR_CODE:
        pass
    elif req.get_status == GetLimoStatusRequest.GET_STATUS_MOTION_MODE:
        pass
    elif req.get_status == GetLimoStatusRequest.GET_STATUS_VEHICLE_STATE:
        pass

    response_msg.status_string = response_string

    return response_msg

def limo_status_callback(msg):
    pass

if __name__ == "__main__":
    rospy.init_node("limo_status_translator_node")

    rospy.Subscriber("limo_status", LimoStatus, limo_status_callback)
    service = rospy.Service("get_limo_status", GetLimoStatus, handle_get_status_req)
    print("Ready to handle get status requests")

    rospy.spin()
#!/usr/bin/env python

import rospy
from lab3_mini_project.srv import GetLimoStatus, GetLimoStatusResponse, GetLimoStatusRequest
from limo_base.msg import LimoStatus

# Empty list to hold processed strings
status_strings = ["Nothing here yet"] * 5

def handle_get_status_req(req):
    response_msg = GetLimoStatusResponse()
    response_msg.status_string = status_strings[req.get_status]

    return response_msg

def limo_status_callback(msg):
    # Process battery voltage
    status_strings[GetLimoStatusRequest.GET_STATUS_BATTERY_VOLTAGE] = " Battery voltage is currently at " + str(msg.battery_voltage) + "V"

    # Process vehicle state
    if msg.vehicle_state != 0: # has errors
        status_strings[GetLimoStatusRequest.GET_STATUS_VEHICLE_STATE] = "Vehicle system state exception"
    else: # No errors
        status_strings[GetLimoStatusRequest.GET_STATUS_VEHICLE_STATE] = "Vehicle system state normal"

    # Process control mode
    if msg.control_mode == 0:
        status_strings[GetLimoStatusRequest.GET_STATUS_CONTROL_MODE] = "Vehicle on Standby"
    elif msg.control_mode == 1:
        status_strings[GetLimoStatusRequest.GET_STATUS_CONTROL_MODE] = "Vehicle controlled by ROS commands"
    elif msg.control_mode == 2:
        status_strings[GetLimoStatusRequest.GET_STATUS_CONTROL_MODE] = "Vehicle controlled by App"
    elif msg.control_mode == 3:
        status_strings[GetLimoStatusRequest.GET_STATUS_CONTROL_MODE] = "Vehicle controlled by Remote"

    # Process motion mode
    if msg.motion_mode == 0:
        status_strings[GetLimoStatusRequest.GET_STATUS_MOTION_MODE] = "Vehicle in 4-wheel differential motion mode"
    elif msg.motion_mode == 1:
        status_strings[GetLimoStatusRequest.GET_STATUS_MOTION_MODE] = "Vehicle in ackerman motion mode"
    elif msg.motion_mode == 2:
        status_strings[GetLimoStatusRequest.GET_STATUS_MOTION_MODE] = "Vehicle in mecanum motion mode"

    # Process error codes
    status_strings[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] = "Error codes present: "
    if msg.error_code & 0b0000000001: # check first bit
        status_strings[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] += "Battery under-voltage fault "
    elif msg.error_code & 0b0000000010:
        status_strings[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] += "Battery under-voltage warning "
    elif msg.error_code & 0b0000000100:
        status_strings[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] += "Remote connection loss "
    elif msg.error_code & 0b0000001000:
        status_strings[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] += "Motor 1 communication failure "
    elif msg.error_code & 0b0000010000:
        status_strings[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] += "Motor 2 communication failure "
    elif msg.error_code & 0b0000100000:
        status_strings[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] += "Motor 3 communication failure "
    elif msg.error_code & 0b0001000000:
        status_strings[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] += "Motor 4 communication failure "
    elif msg.error_code & 0b0100000000:
        status_strings[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] += "Motor driver failure "
    elif msg.error_code & 0b1000000000:
        status_strings[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] += "Upper layer communication failure "

if __name__ == "__main__":
    rospy.init_node("limo_status_translator_node")

    rospy.Subscriber("limo_status", LimoStatus, limo_status_callback)
    service = rospy.Service("get_limo_status", GetLimoStatus, handle_get_status_req)
    print("Ready to handle get status requests")

    rospy.spin()
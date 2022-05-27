#!/usr/bin/env python

import rospy
from lab3_mini_project.srv import GetLimoStatus, GetLimoStatusRequest, GetLimoStatusResponse
from std_msgs.msg import String

def publish_status_strings():
    # Dictionary to store our publishers
    publishers = {}

    # Create and initialize the publishers using the srv constants
    publishers[GetLimoStatusRequest.GET_STATUS_VEHICLE_STATE] = rospy.Publisher("/limo_status/vehicle_state", String, queue_size=1)
    publishers[GetLimoStatusRequest.GET_STATUS_CONTROL_MODE] = rospy.Publisher("/limo_status/control_mode", String, queue_size=1)
    publishers[GetLimoStatusRequest.GET_STATUS_BATTERY_VOLTAGE] = rospy.Publisher("/limo_status/battery_voltage", String, queue_size=1)
    publishers[GetLimoStatusRequest.GET_STATUS_ERROR_CODE] = rospy.Publisher("/limo_status/error_code", String, queue_size=1)
    publishers[GetLimoStatusRequest.GET_STATUS_MOTION_MODE] = rospy.Publisher("/limo_status/motion_mode", String, queue_size=1)

    # Get the service function
    get_status_string = rospy.ServiceProxy("get_limo_status", GetLimoStatus)

    # Set loop rate to 1hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        for key, pub in publishers.items():
            res = get_status_string(key) 
            pub.publish(res.status_string)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("limo_status_client_node")
    rospy.wait_for_service('get_limo_status')
    publish_status_strings()
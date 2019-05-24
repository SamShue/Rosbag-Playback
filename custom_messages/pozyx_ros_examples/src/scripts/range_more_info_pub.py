#!/usr/bin/env python
"""ROS node that publishes the device range between two Pozyx's."""

import pypozyx
import pypozyx.definitions.bitmasks

import rospy
from pozyx_ros_examples.msg import DeviceRange

remote_id = None
destination_id = 0x6036


def pozyx_ranging_pub():
    pub = rospy.Publisher('pozyx_device_range', DeviceRange, queue_size=100)
    rospy.init_node('range_info_pub')
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    while not rospy.is_shutdown():
        device_range = pypozyx.DeviceRange()
        # temporary fix for the issue
        pozyx.checkForFlag(pypozyx.definitions.bitmasks.POZYX_INT_MASK_RX_DATA,
                           pypozyx.POZYX_DELAY_INTERRUPT)
        if pozyx.doRanging(destination_id, device_range, remote_id=remote_id):
            pub.publish(device_range.timestamp,
                        device_range.distance, device_range.RSS)
            rospy.loginfo(device_range)
        else:
            error_code = SingleRegister()
            pozyx.getErrorCode(error_code)
            rospy.loginfo('ERROR: RANGING, error code %s' % error_code)

if __name__ == '__main__':
    try:
        pozyx_ranging_pub()
    except rospy.ROSInterruptException:
pass

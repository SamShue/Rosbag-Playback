#!/usr/bin/env python
"""ROS node that publishes the device range between two Pozyx's."""
import serial.tools.list_ports
import pypozyx
import pypozyx.definitions.bitmasks

import rospy
from pozyx_ros_examples.msg import DeviceRange
from std_msgs.msg import String
remote_id = None
destination_id = [0x6955,0x6940,0x6937,0x6935];

count = 0;

def pozyx_ranging_pub():
    #pub = rospy.Publisher('pozyx_device_range', DeviceRange, queue_size=100)
    pub = rospy.Publisher('pozyx_device_range', DeviceRange, queue_size=100)
    rospy.init_node('range_info_pub')
    r=rospy.Rate(20);
    try:
        #pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
	#print serial.tools.list_ports.comports()[0]
	#pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
        pozyx = pypozyx.PozyxSerial('/dev/ttyACM1')
    except:
        rospy.loginfo("Pozyx not connected")
        return
    while not rospy.is_shutdown():
        device_range = pypozyx.DeviceRange()
        # temporary fix for the issue
        pozyx.checkForFlag(pypozyx.definitions.bitmasks.POZYX_INT_MASK_RX_DATA,
                           pypozyx.POZYX_DELAY_INTERRUPT)

	for x in range(0,4):
           if pozyx.doRanging(destination_id[x], device_range, remote_id=remote_id):
           	pub.publish(device_range.timestamp,device_range.distance, device_range.RSS,str(destination_id[x]))

           	#pub.publish(str(device_range.timestamp)+", "+str(device_range.distance)+", "+str(device_range.RSS)+", "+str(destination_id[x]))
     	        rospy.loginfo(str(device_range)+", "+str(destination_id[x]))

		r.sleep()	
     	   else:
     	        error_code = pypozyx.SingleRegister()
     	        pozyx.getErrorCode(error_code)
     	        rospy.loginfo('ERROR: RANGING, error code %s' %error_code)
		if str(error_code) in '0x1':
     	            rospy.loginfo('ERROR: RANGING, killing publisher due to %04x' %destination_id[x])
     	            return

if __name__ == '__main__':
    try:
        pozyx_ranging_pub()
    except rospy.ROSInterruptException:
	pass

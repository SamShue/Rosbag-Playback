#!/usr/bin/env python
import serial.tools.list_ports
import rospy
import os, sys
import serial
import time


from pozyx_ros_examples.msg import DeviceRange
from std_msgs.msg import String

def pozyx_ranging_pub():
 
    pub = rospy.Publisher('xbee_device_range', DeviceRange, queue_size=100)
    rospy.init_node('range_info_pub')
    r=rospy.Rate(20);
    try:
        #pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
	#print serial.tools.list_ports.comports()[0]
	#pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
        #pozyx = pypozyx.PozyxSerial('/dev/ttyACM1')
	ser = serial.Serial('/dev/ttyUSB0',9600,8,'N',1, timeout = 5)
    except:
        rospy.loginfo("Xbee not connected")
        return
    rospy.loginfo("Xbee succesfully connected")
    while not rospy.is_shutdown():
	line = ser.read(1)
	line=line.encode('hex')
	
	if len(line) == 0:
		print("Time out! Exit.\n")
	elif line[0:2]=='f0':
		line=ser.read(1);
		line=line.encode('hex');
		print "Message recv."
		packetLen=int(line,16);
		data=ser.read(packetLen);
		data=data.encode('hex');
		print data
		#API_IDENTIFIER=line[6:8] 
		#DATA=int(line[8:packetLen+8],32)
		#print "Message received, length:", packetLen,"  data:",DATA, "\n"	
		#print line[packetLen+8:packetLen+8+1]
		#print line

		#process all the data
		timestamp=0.0;
		
		range_rssi=int(data[8:10],32)*-1;
		range_distance=1.0*10.0**((abs(range_rssi)-60.0)/(10*1.8))*100;
		device_id=data[10:18];
		pub.publish(timestamp,range_distance,range_rssi,str(device_id)); 
	#print len(line)



if __name__ == '__main__':
    try:
        pozyx_ranging_pub()
    except rospy.ROSInterruptException:
	pass

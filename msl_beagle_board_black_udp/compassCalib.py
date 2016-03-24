#!/usr/bin/env python
import math
import rospy
from msl_actuator_msgs.msg import IMUData

xmin = 999999999
ymin = 999999999
xmax = -999999999
ymax = -999999999

def callback(data):
	global xmin, xmax, ymin, ymax
	if xmin > data.magnet.x: 
		xmin = data.magnet.x
        if ymin > data.magnet.y:
                ymin = data.magnet.y
        if xmax < data.magnet.x:
                xmax = data.magnet.x
        if ymax < data.magnet.y:
                ymax = data.magnet.y

	curX = data.magnet.x - (xmin + (xmax-xmin)/2)
	curY = data.magnet.y - (ymin + (ymax-ymin)/2)
	print "Angle: ", math.atan2(curY, curX), "\tNormalizedX: ", curX, "\tNormalizedY: ", curY
	print "xMinValue: " , xmin, "\txMaxValue: ", xmax
	print "yMinValue: " , ymin, "\tyMaxValue: ", ymax
	#rospy.loginfo(rospy.get_caller_id() + " %s", xmin)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.temperature)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('IMUCalibration', anonymous=True)

    rospy.Subscriber("/IMUData", IMUData, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()




#!/usr/bin/env python
import math
import rospy
from msl_actuator_msgs.msg import IMUData

xmin = 999999999
ymin = 999999999
zmin = 999999999

xmax = -999999999
ymax = -999999999
zmax = -999999999

def callback(data):
	global xmin, xmax, ymin, ymax, zmin, zmax
	if xmin > data.acceleration.x: 
		xmin = data.acceleration.x
        if ymin > data.acceleration.y:
                ymin = data.acceleration.y
        if zmin > data.acceleration.z:
                zmin = data.acceleration.z
 
        if xmax < data.acceleration.x:
                xmax = data.acceleration.x
        if ymax < data.acceleration.y:
                ymax = data.acceleration.y
        if zmax < data.acceleration.z:
                zmax = data.acceleration.z


	curX = data.acceleration.x - (xmin + (xmax-xmin)/2)
	curY = data.acceleration.y - (ymin + (ymax-ymin)/2)
	print "Angle: ", math.atan2(curY, curX), "\tNormalizedX: ", curX, "\tNormalizedY: ", curY
	print "xMinValue: " , xmin, "\txMaxValue: ", xmax
	print "yMinValue: " , ymin, "\tyMaxValue: ", ymax
	print "zMinValue: " , zmin, "\tzMaxValue: ", zmax
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




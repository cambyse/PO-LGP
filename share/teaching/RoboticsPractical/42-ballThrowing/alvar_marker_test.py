#!/usr/bin/env python
import rospy
import baxter_interface
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose

markers = dict()
keys = [1, 2]

def callback(data):
    markers[data.id] = data.pose.position
    #rospy.loginfo(rospy.get_caller_id() + 'msg: %s', data.points)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('visualization_marker', Marker, callback)
    rospy.spin()

''' Returns squared distance of the two markers defined in keys.
    Returns -1 in case of insufficient data.'''
def get_sq_dist():
    if keys[0] not in markers.keys() or keys[1] not in markers.keys() :
        print('Insufficient data.')
        print('Needed:', keys)
        print('Available:', markers.keys())
        return -1
    else:
        marker1 = markers[keys[0]]
        marker2 = markers[keys[1]]
        return (marker2.x - marker1.x)**2 + (marker2.y - marker1.y)**2

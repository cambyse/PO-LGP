 #!/usr/bin/env python
import rospy
import baxter_interface
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose

markers = dict()

def callback(data):
    markers[data.id] = data.pose.position
    #rospy.loginfo(rospy.get_caller_id() + 'msg: %s', data.points)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('visualization_marker', Marker, callback)
    rospy.spin()

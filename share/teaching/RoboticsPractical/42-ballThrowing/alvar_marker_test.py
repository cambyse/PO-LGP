 #!/usr/bin/env python
import rospy
import baxter_interface
from visualization_msgs.msg import Marker

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'msg: %s', data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('visualization_marker', Marker, callback)
    rospy.spin()

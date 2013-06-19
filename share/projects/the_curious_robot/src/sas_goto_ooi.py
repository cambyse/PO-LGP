#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer
import the_curious_robot.msg as msgs
import util

class GotoOOIActionServer:
    def __init__(self, name):
        self.ooi_id = ""
        self.oois = None
        self.control_done = False
        self.react_to_controller = False

        self.oois_sub = rospy.Subscriber('oois', msgs.Objects, self.oois_cb)
        self.ooi_id_sub = rospy.Subscriber('ooi_id', msgs.ObjectID, self.ooi_id_cb)
        self.control_done_sub = rospy.Subscriber('control_done',
                msgs.control_done, self.control_done_cb)

        self.control_pub = rospy.Publisher('control', msgs.control)

        self.server = SimpleActionServer(name, msgs.GotoOOIAction,
                execute_cb=self.execute)
        self.server.start()

    def execute(self, msg):
        for ooi in self.oois:
            if ooi['body'].name == self.ooi_id:
                msg = msgs.control()
                msg.pose.position.x = ooi['body'].X.pos.x
                msg.pose.position.y = ooi['body'].X.pos.y
                msg.pose.position.z = ooi['body'].X.pos.z
                break

        self.react_to_controller = True #TODO: rather block?
        self.control_pub.publish(msg)
        
        while not self.control_done:
            rospy.sleep(.1)
        self.react_to_controller = False
        self.control_done = False
        self.server.set_succeeded()

    def ooi_id_cb(self, msg):
        self.ooi_id = msg.id

    def oois_cb(self, msg):
        #rospy.logdebug("callback")
        self.oois = util.parse_oois_msg(msg)

    def control_done_cb(self, msg):
        if self.react_to_controller:
            self.control_done = True

def main():
    rospy.init_node('tcr_sas_goto_ooi')
    server = GotoOOIActionServer('goto_ooi')

if __name__ == '__main__':
    main()
    rospy.spin()

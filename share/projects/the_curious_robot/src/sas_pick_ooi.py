#!/usr/bin/env python

import roslib 
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer
import the_curious_robot.msg as msgs
import random
import util

class PickOOIActionServer:
    def __init__(self, name):
        self.oois = None

        # Subscriber
        self.oois_sub = rospy.Subscriber('oois', msgs.Objects, self.oois_cb)
        # right now we don't need the world_belief, that might change
        # self.world_belief_sub = rospy.Subscriber ...

        # Publisher
        self.ooi_id_pub = rospy.Publisher('ooi_id', msgs.ObjectID)

        # Actionlib Server
        self.server = SimpleActionServer(name, msgs.PickOOIAction,
                execute_cb=self.execute)
        self.server.start()

    def execute(self, msg):
        if self.oois is None:
            self.server.set_aborted()
            return

        ooi = random.choice(self.oois)
        ooi_id_msg = msgs.ObjectID()
        ooi_id_msg.id = ooi['body'].name
        self.ooi_id_pub.publish(ooi_id_msg)
        self.server.set_succeeded()

    def oois_cb(self, msg):
        #rospy.logdebug("callback")
        self.oois = util.parse_oois_msg(msg)

def main():
    rospy.init_node('tcr_sas_pick_ooi', log_level=rospy.DEBUG)
    server = PickOOIActionServer('pick_ooi')

if __name__ == '__main__':
    main()
    rospy.spin()

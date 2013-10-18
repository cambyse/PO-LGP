#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer
import the_curious_robot.msg as msgs
import random
import util

import require_provide as rp


#########################################################################
# different strategies for selecting OOIs
#########################################################################
#
# to add new strategies add a method that takes the the list of oois. the
# `select_ooi` in `PickOOIActionServer` must be set to this method.
#
# TODO selecting the strategy should be done via dynamic reconfigure.
def _random_select_strategy(oois):
    """select a random object of all possibes objects"""
    ooi = random.choice(oois)
    ooi_id_msg = msgs.ObjectID()
    ooi_id_msg.id = ooi['body'].name
    return ooi_id_msg


def _door1_select_strategy(oois):
    """always go for the door1-door"""
    ooi_id_msg = msgs.ObjectID()
    ooi_id_msg.id = "door1-door"
    return ooi_id_msg


#########################################################################
class PickOOIActionServer:

    def __init__(self, name):
        # Subscriber
        self.oois_sub = rospy.Subscriber('oois', msgs.Objects, self.oois_cb)
        # right now we don't need the world_belief, that might change
        # self.world_belief_sub = rospy.Subscriber ...

        # Publisher
        self.ooi_id_pub = rospy.Publisher('ooi_id', msgs.ObjectID)

        # Actionlib Server
        self.server = SimpleActionServer(
            name,
            msgs.PickOOIAction,
            execute_cb=self.execute,
            auto_start=False
        )
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        # Select the Strategies
        #self.select_ooi = _random_select_strategy
        self.select_ooi = _random_select_strategy

        self.oois = None
        rp.Provide("PickOOI")

    def execute(self, msg):
        if self.oois is None:
            self.server.set_aborted()
            return

        # select an ooi
        ooi_id_msg = self.select_ooi(self.oois)

        self.ooi_id_pub.publish(ooi_id_msg)
        self.server.set_succeeded()

    def oois_cb(self, msg):
        # rospy.logdebug("callback")
        self.oois = util.parse_oois_msg(msg)

    def preempt_cb(self):
        self.server.set_preempted()


def main():
    rospy.init_node('tcr_sas_pick_ooi')
    server = PickOOIActionServer('pick_ooi')


if __name__ == '__main__':
    main()
    rospy.spin()

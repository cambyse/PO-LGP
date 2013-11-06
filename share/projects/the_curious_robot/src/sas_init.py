#!/usr/bin/env python
# encoding: utf-8

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer

import the_curious_robot.msg as msgs
import the_curious_robot.srv as srvs

import orspy
# import corepy
import util
import require_provide as rp


class InitServer:
    """
    This node initializes all other nodes
    """

    def __init__(self, name):
        self.server = SimpleActionServer(
            name, msgs.EmptyAction,
            execute_cb=self.execute, auto_start=False
        )
        self.oois_pub = rospy.Publisher('oois', msgs.Objects)
        self.all_shapes_pub = rospy.Publisher('all_shapes', msgs.Objects)
        self.belief_pub = rospy.Publisher('world_belief', msgs.ors)

        self.server.start()
        rp.Provide("Init")

    def execute(self, msg):
        rospy.wait_for_service('percept_all')
        self.graph = orspy.Graph()

        # a list of all known shapes of the environment
        all_shapes = []
        oois = []
        try:
            world_init = rospy.ServiceProxy('percept_all', srvs.percept_all)
            world = world_init()
            for p in world.bodies:
                body = util.parse_body_msg(p)
                oois.append({'body': body, 'properties': util.Properties()})
                self.graph.addObject(body)
        except rospy.ServiceException:
            self.server.set_aborted()

        oois_msg = util.create_oois_msg(oois)
        self.oois_pub.publish(oois_msg)
        self.all_shapes_pub.publish(all_shapes)

        belief_msg = msgs.ors()
        belief_msg.ors = str(self.graph)
        self.belief_pub.publish(belief_msg)

        self.server.set_succeeded()


def main():
    rospy.init_node('tcr_sas_init')
    server = InitServer('init')

if __name__ == '__main__':
    main()
    rospy.spin()

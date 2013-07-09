#!/usr/bin/env python

import roslib 
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer

import the_curious_robot.msg as msgs
import the_curious_robot.srv as srvs

import orspy
import util

class InitServer:
    def __init__(self, name):
        self.server = SimpleActionServer(name, msgs.EmptyAction,
                execute_cb=self.execute)
        self.oois_pub = rospy.Publisher('oois', msgs.Objects)
        self.belief_pub = rospy.Publisher('world_belief', msgs.ors)

        self.server.start()

    def execute(self, msg):
        rospy.wait_for_service('percept_all')
        self.graph = orspy.Graph()
        oois = []
        try:
            world_init = rospy.ServiceProxy('percept_all', srvs.percept_all)
            world = world_init()
            for p in world.bodies:
                b = util.parse_body_msg(p)
                oois.append( {'body': b, 'properties': util.Properties()} )
                self.graph.addObject(b)
        except rospy.ServiceException, e:
            self.server.set_aborted()

        oois_msg = util.create_oois_msg(oois)
        self.oois_pub.publish(oois_msg)

        belief_msg = msgs.ors()
        belief_msg.ors = str(self.graph)
        self.belief_pub.publish(belief_msg)

        rospy.logdebug("init done")

        self.server.set_succeeded()

def main():
    rospy.init_node('tcr_sas_init')
    server = InitServer('init')

if __name__ == '__main__':
    main()
    rospy.spin()

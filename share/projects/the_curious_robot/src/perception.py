#!/usr/bin/env python
# encoding: utf-8

import roslib
roslib.load_manifest('the_curious_robot')
import rospy

import the_curious_robot as tcr

import rosors.srv
import rosors
from rosors import parser

import require_provide as rp


class FakePerception():
    """
    The fake perception of the curious robot publishes shapes.
    """

    def __init__(self):
        # init the node: test_fitting
        rospy.init_node('tcr_perception', log_level=rospy.INFO)

        self.graph = None
        self.old_graph = None

        self.update_pub = rospy.Publisher('/perception/updates',
                                          tcr.msg.Objects)

    #########################################################################
    # logic
    def run(self):
        rospy.logdebug("start perception")
        rp.Provide("Perception")
        """ the perception loop """
        while not rospy.is_shutdown():
            self.step()

    def step(self):
        # TODO: newClone() ?
        self.old_graph = self.graph
        graph_srv = rospy.ServiceProxy("/world/graph", rosors.srv.Graph)
        try:
            graph_msg = graph_srv()
        except rospy.ServiceException:
            return

        self.graph = parser.msg_to_ors_graph(graph_msg.graph)

        update_msg = tcr.msg.Objects()
        update_msg.changed = False
        for b in self.graph.bodies:
            if self.has_moved(b):
                update_msg.changed = True
                update_msg.objects.append(b.index)

        #rospy.logdebug(update_msg)
        #rospy.logdebug("="*80)
        self.update_pub.publish(update_msg)

    def has_moved(self, body):
        if self.old_graph is None:
            return True
        old_body = self.old_graph.getBodyByName(body.name)
        eps = 10e-5
        return (body.X.pos - old_body.X.pos).length() > eps


if __name__ == '__main__':
    perception = FakePerception()
    perception.run()

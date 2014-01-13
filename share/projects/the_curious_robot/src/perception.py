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


class FakePerception(object):
    """
    The fake perception of the curious robot publishes shapes.
    """

    def __init__(self):
        # init the node: test_fitting
        rospy.init_node('tcr_perception', log_level=rospy.INFO)

        self.graph = None
        self.eps = 10e-4
        self.update_pub = rospy.Publisher('/perception/updates',
                                          tcr.msg.Objects)

    #########################################################################
    # logic
    def run(self):
        rospy.logdebug("start perception")
        rp.Provide("Perception")
        """ the perception loop """
        self.init()
        while not rospy.is_shutdown():
            self.step()

    def init(self):
        graph_srv = rospy.ServiceProxy("/world/graph", rosors.srv.Graph)
        while not rospy.is_shutdown():
            try:
                graph_msg = graph_srv()
            except rospy.ServiceException:
                continue
            break
        self.graph = parser.msg_to_ors_graph(graph_msg.graph)
        return

    def update_positions(self, body):
        gbody = self.graph.getBodyByName(body.name)
        changed = ((gbody.X.pos - body.X.pos).length() > self.eps)
        #rospy.logdebug(gbody.X)
        #rospy.logdebug(body.X)
        gbody.X = body.X
        return changed

    def step(self):
        try:
            body_srv = rospy.ServiceProxy("/world/bodies", rosors.srv.Bodies)
            body_msg = body_srv(no_shapes=True)
        except rospy.ServiceException:
            return

        update_msg = tcr.msg.Objects()
        update_msg.changed = False
        for body in body_msg.bodies:
            if self.update_positions(parser.msg_to_ors_body(body)):
                update_msg.changed = True
                update_msg.objects += [shape.index for shape in
                                       self.graph.bodies[body.index].shapes]

        self.update_pub.publish(update_msg)


if __name__ == '__main__':
    perception = FakePerception()
    perception.run()

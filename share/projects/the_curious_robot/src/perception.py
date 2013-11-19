#!/usr/bin/env python
# encoding: utf-8

import roslib
roslib.load_manifest('the_curious_robot')
import rospy

import os
import Queue
# import time

# import numpy as np

import the_curious_robot as tcr
import the_curious_robot.srv as srv
import rosors.srv

# The order is important - this sucks
import orspy as ors
import corepy

import require_provide as rp


class FakePerception():
    """
    The fake perception of the curious robot publishes shapes.
    """

    def __init__(self):
        # init the node: test_fitting
        rospy.init_node('tcr_perception')

        self.graph = None
        self.old_graph = None

        self.update_pub = rospy.Publisher('perception/updates', 
                                          rosors.msgs.objects)

    #########################################################################
    # logic
    def run(self):
        rp.Provide("Perception")
        """ the perception loop """
        while not rospy.is_shutdown():
            self.step()

    def step(self):
        # TODO: newClone() ?
        self.old_graph = self.graph
        graph_srv = rospy.ServiceProxy("/world/graph", rosors.srv.Graph)
        graph_msg = graph_srv()

        self.graph = orspy.Graph()

        update_msg = msgs.ObjectIDs()
        update_msg.changed = False
        for b in self.graph.bodies:
            if has_moved(b):
                update_msg.changed = True
                update_msg.objects.append(b.index)

        self.upddate_pub.pub(update_msg)

    def has_moved(self, body):
        if self.not_published_once:
            return True
        old_body = self.old_world.getBodyByName(body.name)
        eps = 10e-5
        return (body.X.pos - old_body.X.pos).length() > eps


if __name__ == '__main__':
    perception = FakePerception()
    perception.run()

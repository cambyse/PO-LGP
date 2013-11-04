#!/usr/bin/env python
# encoding: utf-8

import roslib;
roslib.load_manifest("rosors")

import rospy
import std_msgs.msg
import rosors.srv

import orspy


class RosOrs(object):
    """
    RosOrs represents a ors graph and provieds service calls to query the
    state of the graph.
    """

    def __init__(self, node_name, orsfile):
        # the actual ors graph
        graph = orspy.Graph()
        graph.init(orsfile)

        rospy.init_node()
        # Do i need this here?
        rospy.init_node(node_name)
        self.shape_service = rospy.Service('add_two_ints',
                                           rosors.srv.Shape,
                                           self.handle_shape_request)
        # Do i need this here?
        rospy.spin()

    # handle services
    def handle_shape_request(self, req):
        pass

























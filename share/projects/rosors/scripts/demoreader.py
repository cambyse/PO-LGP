#!/usr/bin/env python
# encoding: utf-8

import roslib
roslib.load_manifest("rosors")

import rospy
import rosors.srv
from rosors import parser


class OrsRosDemoReader():
    """
    This class demonstrates how to use rosors.

    Simply create a rosors member variable.  You need to specify the orsfile
    and the prefix for the service.  The rosors instance automatically offers
    services for the ors data structures.
    """
    def __init__(self):
        node_name = "rosors_test_reader"
        rospy.init_node(node_name, log_level=rospy.DEBUG)

    def run(self):
        while not rospy.is_shutdown():
            self.step()

    def step(self):
        graph_srv = rospy.ServiceProxy("/rosors_demo/graph", rosors.srv.Graph)
        try:
            graph_msg = graph_srv()
        except rospy.ServiceException:
            rospy.logwarn("no service found")
            return

        graph = parser.msg_to_ors_graph(graph_msg.graph)
        rospy.loginfo("read a graph from service")


def main():
    r = OrsRosDemoReader()
    r.run()


if __name__ == '__main__':
    main()

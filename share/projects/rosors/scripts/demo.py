#!/usr/bin/env python
# encoding: utf-8

import roslib
roslib.load_manifest("rosors")

import rospy
import rosors.rosors


class OrsRosDemo():
    """
    This class demonstrates how to use rosors.

    Simply create a rosors member variable.  You need to specify the orsfile
    and the prefix for the service.  The rosors instance automatically offers
    services for the ors data structures.
    """
    def __init__(self):
        orsfile = "arm3.ors"
        node_name = "rosors_test"
        rospy.init_node(node_name, log_level=rospy.DEBUG)

        self.rosors = rosors.rosors.RosOrs(orsfile=orsfile,
                                           srv_prefix="/rosors_demo")


def main():
    OrsRosDemo()
    rospy.spin()


if __name__ == '__main__':
    main()

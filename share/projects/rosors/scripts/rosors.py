#!/usr/bin/env python
# encoding: utf-8

import roslib
roslib.load_manifest("rosors")

import rospy
import rosors.srv

import orspy


class RosOrs(object):
    """
    RosOrs represents a ors graph and provieds service calls to query the
    state of the graph.
    """

    def __init__(self, orsfile, srv_prefix):
        # the actual ors graph
        self.graph = orspy.Graph()
        self.graph.init(orsfile)
        rospy.loginfo("Starting rosors services with prefix: %s " % srv_prefix)

        # start rospy services
        self.body_service = rospy.Service(srv_prefix + "/bodies",
                                          rosors.srv.Bodies,
                                          self.handle_bodies_request)
        self.shape_service = rospy.Service(srv_prefix + "/shapes",
                                           rosors.srv.Bodies,
                                           self.handle_shape_request)

    #########################################################################
    # HANDLE SERVICES
    def handle_bodies_request(self, req):
        rospy.logdebug("handling bodies request")
        res = rosors.srv.BodiesResponse()
        # special body requested
        if req.name:
            ors_body = self.graph.getBodyByName(req.name)
            res.bodies.append(self.ors_body_to_msg(ors_body))
            return res
        # all bodies requested
        for ors_body in self.graph.bodies:
            res.bodies.append(self.ors_body_to_msg(ors_body))
        return res

    def handle_shape_request(self, req):
        raise NotImplementedError("Subclasses should implement this!")

    #########################################################################
    # Helpers for creating msgs
    def ors_body_to_msg(self, ors_body):
        body_msg = rosors.msg.Body()

        body_msg.name = ors_body.name
        body_msg.mass = ors_body.mass

        # ors transformation
        body_msg.pos = ors_body.X.pos
        body_msg.rot = ors_body.X.rot
        body_msg.vel = ors_body.X.vel
        body_msg.angvel = ors_body.X.angvel

        body_msg.com = ors_body.com

        body_msg.force = ors_body.force
        body_msg.torque = ors_body.torque

        return body_msg


class Dummy():
    def __init__(self):
        orsfile = "arm3.ors"
        node_name = "rosors_test"

        rospy.init_node(node_name, log_level=rospy.DEBUG)
        self.rosors = RosOrs(orsfile, "/prefix")


def main():
    rosors = Dummy()
    rospy.spin()


if __name__ == '__main__':
    main()

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

    Supported information:
        - Body
        - Shape
    """

    def __init__(self, orsfile, srv_prefix):
        # the actual ors graph
        self.graph = orspy.Graph()
        self.graph.init(orsfile)
        rospy.loginfo("Starting rosors services with prefix: %s " % srv_prefix)

        # start rospy services
        self.shape_service = rospy.Service(srv_prefix + "/shapes",
                                           rosors.srv.Shapes,
                                           self.handle_shapes_request)
        self.body_service = rospy.Service(srv_prefix + "/bodies",
                                          rosors.srv.Bodies,
                                          self.handle_bodies_request)

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

    def handle_shapes_request(self, req):
        rospy.logdebug("handling shapes request")
        res = rosors.srv.ShapesResponse()
        # special shape requested
        if req.index:
            raise NotImplementedError("get by index is not implemented")
        if req.index_body:
            raise NotImplementedError("get by index_body is not implemented")
        elif req.name:
            ors_shape = self.graph.getShapeByName(req.name)
            res.bodies.append(self.ors_shape_to_msg(ors_shape))
            return res
        # all shapes requested
        for ors_shape in self.graph.shapes:
            res.shapes.append(self.ors_shape_to_msg(ors_shape))
        return res

    #########################################################################
    # Helpers for creating msgs
    def ors_shape_to_msg(self, ors_shape):
        shape_msg = rosors.msg.Shape()

        shape_msg.index = ors_shape.index
        shape_msg.index_body = ors_shape.ibody
        shape_msg.name = ors_shape.name

        shape_msg.X.translation = ors_shape.X.pos
        shape_msg.X.rotation = ors_shape.X.rot
        shape_msg.rel.translation = ors_shape.rel.pos
        shape_msg.rel.rotation = ors_shape.rel.rot

        return shape_msg

    def ors_body_to_msg(self, ors_body):
        body_msg = rosors.msg.Body()

        body_msg.index = ors_body.index
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
        self.rosors = RosOrs(orsfile=orsfile, srv_prefix="/prefix")


def main():
    rosors = OrsRosDemo()
    rospy.spin()


if __name__ == '__main__':
    main()

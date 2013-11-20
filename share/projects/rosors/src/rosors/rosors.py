#!/usr/bin/env python
# encoding: utf-8

# ROS
import roslib
roslib.load_manifest("rosors")
import rospy
# RosOrs
import srv
import parser
# MLR
import orspy


class RosOrs(object):
    """
    RosOrs represents a ors graph and provides ORS services to query the
    state of the graph.

    TODO should we transport subtypes, e.g. transport shapes and meshes for
    a given body? maybe a flag?

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
                                           srv.Shapes,
                                           self.handle_shapes_request)
        self.body_service = rospy.Service(srv_prefix + "/bodies",
                                          srv.Bodies,
                                          self.handle_bodies_request)
        self.graph_service = rospy.Service(srv_prefix + "/graph",
                                           srv.Graph,
                                           self.handle_graph_request)

    #########################################################################
    # HANDLE SERVICES
    def handle_graph_request(self, req):
        rospy.logdebug("handling graph request")
        res = srv.GraphResponse()
        for body in self.graph.bodies:
            res.bodies.append(parser.ors_body_to_msg(body))
        for shape in self.graph.shapes:
            res.shapes.append(parser.ors_shape_to_msg(shape))
        return res

    def handle_bodies_request(self, req):
        rospy.logdebug("handling bodies request")
        res = srv.BodiesResponse()
        # special body requested
        if req.name:
            ors_body = self.graph.getBodyByName(req.name)
            res.bodies.append(parser.ors_body_to_msg(ors_body))
            return res
        # all bodies requested
        for ors_body in self.graph.bodies:
            res.bodies.append(parser.ors_body_to_msg(ors_body))
        return res

    def handle_shapes_request(self, req):
        rospy.logdebug("handling shapes request")
        res = srv.ShapesResponse()
        # special shape requested
        if req.index:
            # TODO ors.graph must support such a function
            raise NotImplementedError("get by index is not implemented")
        if req.index_body:
            # TODO ors.graph must support such a function
            raise NotImplementedError("get by index_body is not implemented")
        elif req.name:
            ors_shape = self.graph.getShapeByName(req.name)
            res.bodies.append(parser.ors_shape_to_msg(ors_shape))
            return res
        # all shapes requested
        for ors_shape in self.graph.shapes:
            res.shapes.append(parser.ors_shape_to_msg(ors_shape))
        return res

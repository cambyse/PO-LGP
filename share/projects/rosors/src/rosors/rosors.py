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
        res = srv.GraphResponse()
        res.graph = parser.ors_graph_to_msg(self.graph)
        return res

    def handle_bodies_request(self, req):
        if req.no_shapes:
            parse = parser.ors_body_to_msg_no_shapes
        else:
            parse = parser.ors_body_to_msg

        res = srv.BodiesResponse()
        # special body requested
        if req.name:
            ors_body = self.graph.getBodyByName(req.name)
            res.bodies.append(parse(ors_body))
            return res
        # all bodies requested
        for ors_body in self.graph.bodies:
            res.bodies.append(parse(ors_body))
        return res

    def handle_shapes_request(self, req):
        res = srv.ShapesResponse()
        # special shape requested
        if req.index:
            ors_shape = self.graph.shapes[req.index]
            res.shapes.append(parser.ors_shape_to_msg(ors_shape,
                                                      req.with_mesh))
            return res
        if req.index_body:
            for ors_shape in self.graph.bodies[req.index_body].shapes:
                res.shapes.append(parser.ors_shape_to_msg(ors_shape,
                                                          req.with_mesh))
            return res
        elif req.name:
            ors_shape = self.graph.getShapeByName(req.name)
            res.shapes.append(parser.ors_shape_to_msg(ors_shape,
                                                      req.with_mesh))
            return res
        # all shapes requested
        for ors_shape in self.graph.shapes:
            res.shapes.append(parser.ors_shape_to_msg(ors_shape,
                                                      req.with_mesh))
        return res

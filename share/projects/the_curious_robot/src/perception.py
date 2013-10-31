#!/usr/bin/env python
# encoding: utf-8

import roslib
roslib.load_manifest('the_curious_robot')
import rospy

import os
import Queue
# import time

# import numpy as np
# import corepy

import the_curious_robot as tcr
import the_curious_robot.srv as srv
import orspy as ors

import require_provide as rp


class FakePerception():
    """
    The fake perception of the curious robot publishes shapes.
    """

    def __init__(self):
        # init the node: test_fitting
        rospy.init_node('tcr_perception')

        self.world = ors.Graph()
        worldfile = os.path.join(
            ors.get_mlr_path(),
            "share/projects/the_curious_robot/src/world.ors"
        )
        self.world.init(worldfile)

        self.worlds = Queue.Queue()
        self.worlds.put(open(worldfile).read())

        self.not_published_once = True

        # Publishers
        self.pub = rospy.Publisher('perception_updates', tcr.msg.percept)
        # Offered Services
        self.percept_service = rospy.Service('percept_all',
                                             srv.percept_all,
                                             self.handle_percept_all_request)
        self.all_shapes_srv = rospy.Service('all_shapes',
                                            srv.all_shapes,
                                            self.handle_all_shapes_request)
        # Subscribers
        self.ors_subs = rospy.Subscriber(name='geometric_state',
                                         data_class=tcr.msg.ors,
                                         callback=self.handle_geometric_state_sub)
        self.frame = 0

    #########################################################################
    # callbacks: handle requests and subscriptions
    def handle_all_shapes_request(self, req):
        rospy.loginfo("all shapes requested")
        response = tcr.srv.all_shapes()
        for shape in self.world.shapes:
            response.shapes.append(str(shape))
        return response

    def handle_percept_all_request(self, req):
        msg = tcr.srv.percept_allResponse()
        msg.header.stamp = rospy.get_rostime()
        for body in self.world.bodies:
            msg.bodies.append(body.name + " " + str(body))
        return msg

    def handle_geometric_state_sub(self, data):
        # simply backup ors data in a queue
        self.worlds.put(data.ors)

    #########################################################################
    # logic
    def run(self):
        rp.Provide("Perception")
        """ the perception loop """
        while not rospy.is_shutdown():
            self.step()

    def step(self):
        if self.worlds.empty():
            return
        self.old_world = self.world.newClone()  # backup for change detection
        self.world.read(self.worlds.get())
        self.world.calcShapeFramesFromBodies()  # don't use
                                                # calcBodyFramesFromJoints
                                                # here. But why?
        agent = self.world.getBodyByName("robot")
        msg = tcr.msg.percept()
        msg.changed = False

        self.frame = self.frame + 1
        for p in self.world.bodies:
            if agent.index != p.index and self.has_moved(p):
                msg.header.stamp = rospy.get_rostime()
                msg.header.seq = self.frame

                msg.bodies.append(p.name + " " + str(p))
                msg.changed = True
                rospy.logdebug(p.name)

        self.not_published_once = False
        self.pub.publish(msg)

    def has_moved(self, body):
        if self.not_published_once:
            return True
        old_body = self.old_world.getBodyByName(body.name)
        eps = 10e-5
        return (body.X.pos - old_body.X.pos).length() > eps


if __name__ == '__main__':
    perception = FakePerception()
    perception.run()

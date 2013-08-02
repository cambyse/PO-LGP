#!/usr/bin/env python

"""
The fake perception of the curious robot.
"""

import roslib
roslib.load_manifest('the_curious_robot')
import rospy
import the_curious_robot.msg as msgs
import the_curious_robot.srv as srvs
# import numpy as np
import os
import orspy as ors
import Queue
# import corepy
# import time

import require_provide as rp


class FakePerception():

    """
    The actual behavior of the robot.
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

        self.pub = rospy.Publisher('perception_updates', msgs.percept)
        self.serv = rospy.Service(
            'percept_all', srvs.percept_all, self.percept_all_cb)
        self.ors_subs = rospy.Subscriber(name='geometric_state',
                                         data_class=msgs.ors,
                                         callback=self.ors_cb)

        self.frame = 0

    def percept_all_cb(self, req):
        msg = srvs.percept_allResponse()
        msg.header.stamp = rospy.get_rostime()
        for p in self.world.bodies:  # TODO: synchronize!
            msg.bodies.append(p.name + " " + str(p))
        return msg

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
                                                # here
        agent = self.world.getBodyByName("robot")
        msg = msgs.percept()
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

    def ors_cb(self, data):
        self.worlds.put(data.ors)  # simply backup ors data in a queue

    def has_moved(self, body):
        if self.not_published_once:
            return True
        old_body = self.old_world.getBodyByName(body.name)
        eps = 10e-5
        return (body.X.pos - old_body.X.pos).length() > eps


if __name__ == '__main__':
    perception = FakePerception()
    perception.run()

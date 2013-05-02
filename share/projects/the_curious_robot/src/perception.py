#!/usr/bin/env python

"""
The fake perception of the curious robot.

NOTE: this does not work yet!
"""

import roslib
roslib.load_manifest('the_curious_robot')
import rospy
import the_curious_robot.msg as msgs
# import numpy as np
import os
import orspy as ors
import Queue
import time


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

        #self.gl = ors.OpenGL()
        #ors.bindOrsToOpenGL(self.world, self.gl)

        self.pub = rospy.Publisher('perception_updates', msgs.percept)
        self.ors_subs = rospy.Subscriber(name='geometric_state',
                                         data_class=msgs.ors,
                                         callback=self.ors_cb)

    def run(self):
        """ the perception loop """
        while not rospy.is_shutdown():
            self.step()

    def step(self):
        if self.worlds.empty():
            return
        self.old_world = self.world             # backup for change detection
        self.world.read(self.worlds.get())
        self.world.calcShapeFramesFromBodies()  # don't use
                                                # calcBodyFramesFromJoints
                                                # here
        agent = self.world.getBodyByName("robot")
        msg = msgs.percept()
        msg.changed = False
        for p in self.world.bodies:
            if agent.index is not p.index and self.has_moved(p):

                msg.bodies.append(p.name + " " + str(p))
                msg.changed = True

        #if self.not_published_once:
            #print msg

        self.not_published_once = False
        self.pub.publish(msg)
        #self.gl.update()

    def ors_cb(self, data):
        self.worlds.put(data.ors)  # simply backup ors data in a queue

    def has_moved(self, body):
        if self.not_published_once:
            return True
        old_body = self.old_world.getBodyByName(body.name)
        return body.X == old_body.X


if __name__ == '__main__':
    perception = FakePerception()
    perception.run()

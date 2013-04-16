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

        self.gl = ors.OpenGL()
        ors.bindOrsToOpenGL(self.world, self.gl)

        self.pub = rospy.Publisher('perception_updates', msgs.percept)

    def run(self):
        """ the perception loop """
        while True:
            self.step()

    def step(self):
        agent = self.world.getBodyByName("robot")
        for p in self.world.bodies:
            if agent.index is not p.index:
                msg = msgs.percept()
                msg.body = str(p)
                self.pub.publish(msg)
        self.gl.update()


if __name__ == '__main__':
    perception = FakePerception()
    perception.run()

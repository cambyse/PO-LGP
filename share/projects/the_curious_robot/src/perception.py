#!/usr/bin/env python

"""
The behavior of the curious robot.

NOTE: this does not work yet!
"""

import roslib
roslib.load_manifest('the_curious_robot')
import rospy
import numpy as np

import sys
sys.path.append('/home/johannes/mlr/git/share/lib/')
import orspy as ors


class FakePerception():
    """
    The actual behavior of the robot.
    """
    def __init__(self):
        # init the node: test_fitting
        rospy.init_node('tcr_perception')

        self.world = ors.Graph()
        self.world.init("/home/johannes/mlr/git/share/projects/the_curious_robot/src/doorSimple.ors")

        self.gl = ors.OpenGL()
        ors.bindOrsToOpenGL(self.world, self.gl)

        self.percepts = ors.ArrayDouble()

    def run(self):
        """ the perception loop """
        while True:
            self.step()

    def step(self):
        agent = self.world.getBodyByName("robot");
        for p in self.world.bodies:
            if agent.index is not p.index:
                self.percepts.append(p.X.pos.x)
                self.percepts.append(p.X.pos.y)
                self.percepts.append(p.X.pos.z)
        self.gl.update()

if __name__ == '__main__':
    perception = FakePerception()
    perception.run()

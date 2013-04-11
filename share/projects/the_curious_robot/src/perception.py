#!/usr/bin/env python

"""
The fake perception of the curious robot.

NOTE: this does not work yet!
"""

import roslib
roslib.load_manifest('the_curious_robot')
import rospy
import the_curious_robot.msg as msgs
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

    def run(self):
        """ the perception loop """
        while True:
            self.step()

    def step(self):
        agent = self.world.getBodyByName("robot");
        for p in self.world.bodies:
            if agent.index is not p.index:
                msg = msgs.percept()
                msg.obj_name = p.name
                msg.new_pose.position.x = p.X.pos.x
                msg.new_pose.position.y = p.X.pos.y
                msg.new_pose.position.z = p.X.pos.z
                msg.new_pose.orientation.x = p.X.rot.x
                msg.new_pose.orientation.y = p.X.rot.y
                msg.new_pose.orientation.z = p.X.rot.z
                msg.new_pose.orientation.w = p.X.rot.w
        self.gl.update()

if __name__ == '__main__':
    perception = FakePerception()
    perception.run()

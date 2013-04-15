#!/usr/bin/env python

"""
The fake controller of the curious robot.

NOTE: this does not work yet!
"""

import roslib
roslib.load_manifest('the_curious_robot')
import rospy
import the_curious_robot.msg as msgs
# import numpy as np
import orspy as ors


class FakeController():
    """
    The actual behavior of the robot.
    """
    def __init__(self):
        # init the node: test_fitting
        rospy.init_node('tcr_controller')

        self.world = ors.Graph()
        self.world.init("/home/johannes/mlr/git/share/projects/the_curious_robot/src/doorSimple.ors")

        self.gl = ors.OpenGL()
        ors.bindOrsToOpenGL(self.world, self.gl)

        self.pub = rospy.Publisher('geometric_state', msgs.ors)
        self.traj_sub = rospy.Subscriber(name='control',
                                         data_class=msgs.control,
                                         callback=self.control_cb)

        self.goal = None

    def run(self):
        """ the controller loop """
        while True:
            self.step()

    def step(self):
        # P-Controller
        if self.goal:
            Kp = 10e-3
            agent = self.world.getBodyByName("robot")
            agent.X.pos = agent.X.pos + (self.goal.pos - agent.X.pos)*Kp
            self.world.calcBodyFramesFromJoints()
            #agent.X.rot = agent.X.rot + (self.goal.rot - agent.X.rot)*Kp

        msg = msgs.ors()
        msg.ors = str(self.world)
        self.pub.publish(msg)
        self.gl.update()

    def control_cb(self, data):
        print "Got control message."
        if not self.goal:
            self.goal = ors.Transformation()

        self.goal.pos.x = data.pose.position.x
        self.goal.pos.y = data.pose.position.y
        self.goal.pos.z = data.pose.position.z
        #self.goal.rot = data.pose.orientation


if __name__ == '__main__':
    controller = FakeController()
    controller.run()

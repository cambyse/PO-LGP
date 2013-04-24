#!/usr/bin/env python

"""
The fake controller of the curious robot.

NOTE: this does not work yet!
"""

import roslib
roslib.load_manifest('the_curious_robot')
import rospy
import the_curious_robot.msg as msgs
import os
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
        worldfile = os.path.join(
            ors.get_mlr_path(),
            "share/projects/the_curious_robot/src/world.ors"
        )
        self.world.init(worldfile)

        self.gl = ors.OpenGL()
        self.physx = ors.PhysXInterface()
        ors.bindOrsToPhysX(self.world, self.gl, self.physx)

        self.pub = rospy.Publisher('geometric_state', msgs.ors)
        self.traj_sub = rospy.Subscriber(name='control',
                                         data_class=msgs.control,
                                         callback=self.control_cb)
        self.goal = ors.Transformation()
        self.frame_id = 1

    def run(self):
        """ the controller loop """
        while not rospy.is_shutdown():
            self.step()

    def step(self):
        # P-Controller
        if self.goal:
            Kp = 10e-2
            agent = self.world.getBodyByName("robot")
            agent.X.pos = agent.X.pos + (self.goal.pos - agent.X.pos) * Kp
            #agent.X.rot = agent.X.rot + (self.goal.rot - agent.X.rot)*Kp

        self.world.calcBodyFramesFromJoints()
        self.physx.step()  # this MUST be before serializing!

        msg = msgs.ors()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = str(self.frame_id)
        self.frame_id = self.frame_id + 1
        msg.ors = str(self.world)
        self.pub.publish(msg)

        self.gl.update()

    def control_cb(self, data):
        print "Got control message."
        self.goal.pos.x = data.pose.position.x
        self.goal.pos.y = data.pose.position.y
        self.goal.pos.z = data.pose.position.z
        #self.goal.rot = data.pose.orientation


if __name__ == '__main__':
    controller = FakeController()
    controller.run()

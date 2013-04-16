#!/usr/bin/env python

"""
This represents the world the robot lives in.
"""

import roslib
roslib.load_manifest('the_curious_robot')
import rospy
import the_curious_robot.msg as msgs
# import numpy as np
import os
import orspy as ors


class World():
    """
    World is a ors instance which does a few thingt
    - publish the world state
    - change the world state according to the control of the robot
    """
    def __init__(self):
        # init the node
        rospy.init_node('tcr_world')

        # ors graph
        self.world = ors.Graph()
        worldfile = os.path.join(
            ors.get_mlr_path(),
            "share/projects/the_curious_robot/src/world.ors"
        )
        self.world.init(worldfile)
        self.agent = self.world.getBodyByName("robot")

        # visualization and physics
        self.gl = ors.OpenGL()
        self.physx = ors.PhysXInterface()
        ors.bindOrsToPhysX(self.world, self.gl, self.physx)

        # pub and sub
        self.perception_pub = rospy.Publisher('perception_updates',
                                              msgs.percept)
        self.traj_sub = rospy.Subscriber(name='control',
                                         data_class=msgs.control,
                                         callback=self.control_cb)

    def run(self):
        """ the perception loop """
        while True:
            self.step()

    def step(self):
        # update ors
        self.physx.step()
        self.gl.update()
        # puplish
        agent = self.world.getBodyByName("robot")
        for p in self.world.bodies:
            if agent.index is not p.index:
                msg = msgs.percept()
                msg.body = str(p)
                self.perception_pub.publish(msg)

    def control_cb(self, data):
        print "Got control message."
        # move agent
        # TODO maybe the P-controller should not be part of this
        # TODO maybe just set the data here
        # TODO maybe update the simulator here like in step()
        Kp = 10e-3
        self.agent.X.pos.x = self.agent.X.pos.y +\
            (data.pose.position.x - self.agent.X.pos.x) * Kp
        self.agent.X.pos.y = self.agent.X.pos.y +\
            (data.pose.position.y - self.agent.X.pos.y) * Kp
        self.agent.X.pos.z = self.agent.X.pos.z +\
            (data.pose.position.z - self.agent.X.pos.z) * Kp
        self.world.calcBodyFramesFromJoints()
        self.physx.step()
        self.gl.update()
        # self.goal.pos.x = data.pose.position.x
        # self.goal.pos.y = data.pose.position.y
        # self.goal.pos.z = data.pose.position.z
        #self.goal.rot = data.pose.orientation


if __name__ == '__main__':
    world = World()
    world.run()

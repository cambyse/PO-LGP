#!/usr/bin/env python

"""
The fake controller of the curious robot.
"""

import roslib
roslib.load_manifest('the_curious_robot')

import rospy
import the_curious_robot.msg as msgs
import the_curious_robot.srv as srvs
import os

import rosors.rosors
import rosors.srv

# swig wrapper
import orspy as ors
import guipy
import motionpy
import corepy

import require_provide as rp
import numpy as np


class FakeController():

    """
    The actual behavior of the robot.
    """
    def __init__(self):
        # init the node: test_fitting
        rospy.init_node('tcr_controller')

        # World & PhysX & OpenGL
        worldfile = os.path.join(
            corepy.get_mlr_path(),
            "share/projects/the_curious_robot/src/world.ors"
        )
        self.world = rosors.rosors.RosOrs(orsfile=worldfile,
                                          srv_prefix="/world")

        self.gl = guipy.OpenGL()
        self.physx = ors.PhysXInterface()
        ors.bindOrsToPhysX(self.world.graph, self.gl, self.physx)

        # Subscriber & Publisher
        self.control_done_pub = rospy.Publisher(
            'control_done', msgs.control_done)
        self.traj_sub = rospy.Subscriber(
            name='control',
            data_class=msgs.control,
            callback=self.control_cb)
        self.control_service = rospy.Service("control", srvs.Control,
                                             self.control_service)

        # Misc
        self.goal = None
        self.frame_id = 1
        self.recompute_trajectory = False
        self.trajectory = None

    def run(self):
        rp.Provide("Controller")
        """ the controller loop """
        while not rospy.is_shutdown():
            self.step()

    def compute_trajectory(self):
        rospy.loginfo("Compute a new Trajectory")
        P = motionpy.MotionProblem(self.world.graph)
        P.T = 1

        shapes = np.ndarray([self.world.graph.getBodyByName("robot").shapes[0]
                             .index],
                            np.uint32)
        c = P.addTaskMap("proxyColls",
                         motionpy.ProxyTaskMap(motionpy.allVersusListedPTMT,
                                               shapes, .01, True))
        P.setInterpolatingCosts(c, motionpy.MotionProblem.constant,
                                np.ndarray([0]), 1e-0)
        planner = motionpy.RRTPlanner(self.world.graph, P, self.stepsize)

        planner.joint_max = np.ndarray([6, 6, 1])
        planner.joint_max = np.ndarray([-6, -6, 1])

        target = corepy.ARRAY(self.goal.pos)
        self.trajectory = planner.getTrajectoryTo(target, .01)

        rospy.logdebug(self.trajectory)

    def step(self):
        if self.recompute_trajectory:
            self.compute_trajectory()

        if self.trajectory:
            self.world.graph.setJointState(self.trajectory[self.tpos])
            self.tpos += 1

        self.physx.step()
        self.world.graph.calcBodyFramesFromJoints()
        self.gl.update()

    def pstep(self):
        # P-Controller
        Kp = 10e-3
        # tolerance for he movement
        eps = 10e-3
        agent = self.world.graph.getBodyByName("robot")

        if self.goal:
            if (agent.X.pos - self.goal.pos).length() > eps:

                agent.X.pos = (agent.X.pos +
                               (self.goal.pos - agent.X.pos) * Kp)

            else:
                msg = msgs.control_done()
                msg.header.frame_id = 'control done'
                self.control_done_pub.publish(msg)
                self.goal = None

        self.physx.step()
        self.world.graph.calcBodyFramesFromJoints()
        self.gl.update()

    def control_cb(self, data):
        new_goal = corepy.Transformation()
        new_goal.pos.x = data.pose.translation.x
        new_goal.pos.y = data.pose.translation.y
        new_goal.pos.z = data.pose.translation.z
        new_goal.rot.x = data.pose.rotation.x
        new_goal.rot.y = data.pose.rotation.y
        new_goal.rot.z = data.pose.rotation.z
        new_goal.rot.w = data.pose.rotation.w

        if self.goal is None:
            self.goal = new_goal
            self.recompute_trajectory = True

    def control_service(self, req):
        new_goal = corepy.Transformation()
        new_goal.pos.x = req.pose.translation.x
        new_goal.pos.y = req.pose.translation.y
        new_goal.pos.z = req.pose.translation.z
        new_goal.rot.x = req.pose.rotation.x
        new_goal.rot.y = req.pose.rotation.y
        new_goal.rot.z = req.pose.rotation.z
        new_goal.rot.w = req.pose.rotation.w

        self.goal = new_goal
        self.recompute_trajectory = True

        while self.goal is not None:
            self.step()

        return srvs.ControlResponse()


if __name__ == '__main__':
    controller = FakeController()
    controller.run()

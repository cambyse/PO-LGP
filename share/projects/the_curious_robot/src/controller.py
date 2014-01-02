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


class RRTPlanner():
    def __init__(self, graph):
        self.graph = graph
        shapes = ors.makeConvexHulls(self.graph.shapes)
        self.graph.shapes = shapes

    def create_endpose(self, target, endeff, start_q=None,
                       col_prec=1e3,
                       col_shapes=np.array([], dtype=np.uint32),
                       pos_prec=1e1):
        rospy.logdebug("start calculating endpose")
        start = self.graph.getJointState()
        if start_q is not None:
            self.graph.setJointState(start_q)
        else:
            start_q = self.graph.getJointState()

        problem = motionpy.MotionProblem(self.graph)
        problem.loadTransitionParameters()
        problem.H_rate_diag = corepy.getArrayParameter("Hratediag")

        rospy.logdebug(self.graph.getJointStateDimension())

        if col_prec != 0:
            proxy_tm = motionpy.ProxyTaskMap(motionpy.allVersusListedPTMT,
                                             col_shapes, 0.01, True)
            task_cost2 = problem.addTaskMap("proxyColls", proxy_tm)
            problem.setInterpolatingCosts(task_cost2,
                                          motionpy.MotionProblem.constant,
                                          np.array([0]), col_prec)

        if pos_prec != 0:
            position_tm = motionpy.DefaultTaskMap(motionpy.posTMT, self.graph,
                                                  endeff,
                                                  corepy.Vector(0, 0, 0))
            task_cost = problem.addTaskMap("position", position_tm)
            problem.setInterpolatingCosts(task_cost,
                                          motionpy.MotionProblem.finalOnly,
                                          target, 1e2)
            problem.setInterpolatingVelCosts(task_cost, motionpy.
                                             MotionProblem.finalOnly,
                                             np.array([0., 0., 0.]), pos_prec)

        _, x = motionpy.keyframeOptimizer(start_q, problem, True, 0)

        self.graph.setJointState(start)
        rospy.logdebug("done calculating endpose")
        return x

    def create_rrt_trajectory(self, target, endeff, collisions=True,
                              col_shapes=np.array([], dtype=np.uint32)):
        rospy.logdebug("start calculating rrt trajectory")
        stepsize = .01

        problem = motionpy.MotionProblem(self.graph)
        problem.loadTransitionParameters()

        if collisions:
            proxy_tm = motionpy.ProxyTaskMap(motionpy.allVersusListedPTMT,
                                             col_shapes, .01, True)
            task_cost = problem.addTaskMap("proxyColls", proxy_tm)

            problem.setInterpolatingCosts(task_cost,
                                          motionpy.MotionProblem.constant,
                                          np.array([0]), 1e-0)
            task_cost.y_threshold = 0

        planner = motionpy.RRTPlanner(self.graph, problem, stepsize)

        planner.joint_max = corepy.getArrayParameter("joint_max")
        planner.joint_min = corepy.getArrayParameter("joint_max")

        traj = planner.getTrajectoryTo(target)
        rospy.logdebug("done calculating rrt trajectory")
        return traj


class FakeController():
    def __init__(self):
        # init the node: test_fitting
        rospy.init_node('tcr_controller', log_level=rospy.DEBUG)

        # World & PhysX & OpenGL
        orspath = "share/projects/the_curious_robot/src"
        orsfile = corepy.getStringParameter("orsFile")
        worldfile = os.path.join(
            corepy.get_mlr_path(), orspath, orsfile
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

        self.rrt = RRTPlanner(self.world.graph)

    def run(self):
        rp.Provide("Controller")
        """ the controller loop """
        while not rospy.is_shutdown():
            self.step()

    def compute_trajectory(self):
        rospy.loginfo("start computing trajectory")

        if self.collisions:
            # first create a target without collision detection
            # this leads us to a penetrating positions
            target = self.rrt.create_endpose(target=corepy.ARRAY(self.goal.
                                                                 pos),
                                             endeff=self.endeff,
                                             col_prec=0,
                                             col_shapes=self.collision_shapes,
                                             pos_prec=1e3)

            # if we want to avoid collisions we then reoptimize mainly on the
            # collisions, that gives us a position very close to the object
            target = self.rrt.create_endpose(target=corepy.ARRAY(self.goal.
                                                                 pos),
                                             endeff=self.endeff,
                                             start_q=target,
                                             col_prec=1e3,
                                             col_shapes=self.collision_shapes,
                                             pos_prec=0)
        else:
            target = self.rrt.create_endpose(target=corepy.ARRAY(self.goal.
                                                                 pos),
                                             endeff=self.endeff,
                                             col_prec=0,
                                             pos_prec=1e3)

        if self.teleport:
            self.trajectory = np.reshape(target, (1, target.shape[0]))
        else:
            self.trajectory = self.rrt.create_rrt_trajectory(target=target,
                                                             endeff=
                                                             self.endeff,
                                                             collisions=
                                                             self.collisions)

        self.tpos = 0
        self.recompute_trajectory = False

        rospy.loginfo("done computing trajectory")

    def step(self):
        if self.recompute_trajectory and self.goal:
            self.compute_trajectory()
            # rospy.loginfo(self.trajectory)
            # rospy.loginfo("new trajectory")

        if self.trajectory is not None:
            # rospy.loginfo("Move one step")
            if self.tpos == self.trajectory.shape[0]:
                self.control_done()
            else:
                # rospy.logdebug("next step: " +
                #                str(self.trajectory[self.tpos, :]))
                self.world.graph.setJointState(self.trajectory[self.tpos, :])
                self.world.graph.calcBodyFramesFromJoints()
                self.tpos += 1

        self.physx.step()
        self.world.graph.calcBodyFramesFromJoints()
        self.gl.update()

    def control_done(self):
        self.trajectory = None
        msg = msgs.control_done()
        msg.header.frame_id = 'control done'
        self.control_done_pub.publish(msg)
        self.goal = None

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

        rospy.loginfo("Set new goal")

        self.endeff = data.endeffector

        self.collisions = not data.ignore_collisions
        self.collision_shapes = np.array(data.collision_shapes,
                                         dtype=np.uint32)

        self.goal = new_goal
        self.recompute_trajectory = True
        self.teleport = data.teleport

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
            rospy.sleep(.1)  # Block till done

        return srvs.ControlResponse()


if __name__ == '__main__':
    controller = FakeController()
    controller.run()

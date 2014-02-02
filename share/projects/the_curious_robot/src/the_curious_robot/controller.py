#!/usr/bin/env python2

import roslib
roslib.load_manifest('the_curious_robot')
import rospy

import orspy as ors
import motionpy as motion
import optimpy as optim
import guipy as gui
import corepy as core

import numpy as np

import the_curious_robot.srv as srvs
import the_curious_robot.msg as msgs

import rosors.rosors
import rosors.srv
import util

import require_provide as rp

import random
import os

class TrajectoryException(Exception):
    pass


class RRTPlanner(object):
    def __init__(self, graph):
        print "init rrt planner"
        self.graph = graph
        shapes = ors.makeConvexHulls(self.graph.shapes)
        self.graph.shapes = shapes

    def is_feasible(self, pose, col_prec, pos_prec, goal):
        problem = motion.MotionProblem(self.graph)
        problem.loadTransitionParameters()

        if col_prec != 0:
            shapes = self.graph.getShapeIdxByAgent(0)
            proxy_tm = motion.ProxyTaskMap(motion.allVersusListedPTMT,
                                           shapes,
                                           .05,
                                           True)
            task_cost2 = problem.addTaskMap("proxyColls", proxy_tm)
            problem.setInterpolatingCosts(task_cost2,
                                          motion.MotionProblem.constant,
                                          np.array([0]), col_prec)

        if pos_prec != 0:
            position_tm = motion.DefaultTaskMap(motion.posTMT,
                                                self.graph,
                                                "eff",
                                                core.Vector(0, 0, 0))
            task_cost = problem.addTaskMap("position", position_tm)
            problem.setInterpolatingCosts(task_cost,
                                          motion.MotionProblem.finalOnly,
                                          core.ARRAY(goal),
                                          pos_prec)
            problem.setInterpolatingVelCosts(task_cost, motion
                                             .MotionProblem.finalOnly,
                                             np.array([0., 0., 0.]), 1e1)

        #problem.setState(pose)
        feasiblity, _, _, _ = problem.getTaskCosts(np.array([]), np.array([]),
                                                   np.array([]), 0)
        return feasiblity

    def create_endpose(self, start, col_prec, pos_prec, endeff, goal):
        problem = motion.MotionProblem(self.graph)
        problem.loadTransitionParameters()

        if col_prec != 0:
            shapes = self.graph.getShapeIdxByAgent(0)
            proxy_tm = motion.ProxyTaskMap(motion.allVersusListedPTMT,
                                           shapes,
                                           .1,
                                           True)
            task_cost2 = problem.addTaskMap("proxyColls", proxy_tm)
            problem.setInterpolatingCosts(task_cost2,
                                          motion.MotionProblem.constant,
                                          np.array([0]), col_prec)

        if pos_prec != 0:
            position_tm = motion.DefaultTaskMap(motion.posTMT,
                                                self.graph,
                                                endeff,
                                                core.Vector(0, 0, 0))
            task_cost = problem.addTaskMap("position", position_tm)
            problem.setInterpolatingCosts(task_cost,
                                          motion.MotionProblem.finalOnly,
                                          core.ARRAY(goal),
                                          pos_prec)
            problem.setInterpolatingVelCosts(task_cost, motion
                                             .MotionProblem.finalOnly,
                                             np.array([0., 0., 0.]), 1e1)

        _, x = motion.keyframeOptimizer(start, problem, True, 0)

        return x

    def create_rrt_trajectory(self, target, collisions):
        stepsize = core.getDoubleParameter("rrt_stepsize", .005)

        problem = motion.MotionProblem(self.graph)
        problem.loadTransitionParameters()

        if collisions:
            shapes = self.graph.getShapeIdxByAgent(0)
            proxy_tm = motion.ProxyTaskMap(motion.allVersusListedPTMT, shapes,
                                           .01,
                                           True)
            task_cost = problem.addTaskMap("proxyColls", proxy_tm)

            problem.setInterpolatingCosts(task_cost,
                                          motion.MotionProblem.constant,
                                          np.array([0]), 1e-0)
            task_cost.y_threshold = 0

        planner = motion.RRTPlanner(self.graph, problem, stepsize, True)
        planner.joint_max = core.getArrayParameter("joint_max")
        planner.joint_min = core.getArrayParameter("joint_min")

        return planner.getTrajectoryTo(target, 10000)

    def optimize_trajectory(self, trajectory, collisions, endeff,
                            goal):
        problem = motion.MotionProblem(self.graph)
        problem.loadTransitionParameters()
        problem.T = trajectory.shape[0]-1

        if collisions:
            shapes = self.graph.getShapeIdxByAgent(0)
            proxy_tm = motion.ProxyTaskMap(motion.allVersusListedPTMT,
                                           shapes,
                                           .05,
                                           True)
            task_cost2 = problem.addTaskMap("proxyColls", proxy_tm)
            problem.setInterpolatingCosts(task_cost2,
                                          motion.MotionProblem.constant,
                                          np.array([0]), 1e1)

        position_tm = motion.DefaultTaskMap(motion.posTMT,
                                            self.graph,
                                            endeff,
                                            core.Vector(0, 0, 0))

        task_cost = problem.addTaskMap("position", position_tm)
        problem.setInterpolatingCosts(task_cost,
                                      motion.MotionProblem.finalOnly,
                                      core.ARRAY(goal),
                                      1e3)
        problem.setInterpolatingVelCosts(task_cost, motion
                                         .MotionProblem.finalOnly,
                                         np.array([0., 0., 0.]), 1e1)

        mp_function = motion.MotionProblemFunction(problem)
        x = trajectory
        options = optim.OptOptions()
        options.verbose = 2
        options.stopIters = 40
        options.useAdaptiveDamping = False
        options.damping = 1e-0
        options.maxStep = 1.
        optim.optNewton(x, optim.Convert(mp_function).asScalarFunction(),
                        options)
        return x


class FakeController(object):
    def __init__(self):
        print "init controller"
        # init the node: test_fitting
        rospy.init_node('tcr_controller', log_level=rospy.DEBUG)

        # World & PhysX & OpenGL
        print "get path"
        orspath = "share/projects/the_curious_robot/src/the_curious_robot"
        orsfile = core.getStringParameter("orsFile")
        worldfile = os.path.join(
            core.get_mlr_path(), orspath, orsfile
        )
        self.world = rosors.rosors.RosOrs(orsfile=worldfile,
                                          srv_prefix="/world")
        self.gl = gui.OpenGL()
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
        self.max_tries = 10

        self.rrt = RRTPlanner(self.world.graph)

    def run(self):
        print "Controller up and running"
        rp.Provide("Controller")
        """ the controller loop """
        while not rospy.is_shutdown():
            self.step()

    def compute_trajectory(self):
        rospy.loginfo("start computing trajectory")

        start = self.rrt.graph.getJointState()
        print start

        # this is pr2-only :(
        opt_start = self.rrt.graph.getJointState()
        opt_start[0] = self.goal.pos.x
        opt_start[1] = self.goal.pos.y

        if self.collisions:
            col_prec = 1e1
        else:
            col_prec = 0

        feasible = False
        for i in range(self.max_tries):
            opt_start[2] = random.uniform(-np.pi, np.pi)

            target = self.rrt.create_endpose(opt_start,
                                             col_prec=col_prec,
                                             pos_prec=1e3,
                                             goal=self.goal.pos,
                                             endeff=self.endeff)
            if self.collisions:
                target2 = self.rrt.create_endpose(target,
                                                  col_prec=1e3,
                                                  pos_prec=1e1,
                                                  goal=self.goal.pos,
                                                  endeff=self.endeff)
            else:
                target2 = target

            feasible = self.rrt.is_feasible(target2,
                                            col_prec=col_prec,
                                            pos_prec=0,
                                            goal=self.goal.pos)
            if feasible:
                break

        if not feasible:
            self.trajectory = None
            self.rrt.graph.setJointState(start)
            self.rrt.graph.calcBodyFramesFromJoints()
            self.rrt.graph.calcBodyFramesFromJoints()
            raise TrajectoryException()

        if self.teleport:
            self.trajectory = np.reshape(target2, (1, target2.shape[0]))
        else:
            self.rrt.graph.setJointState(start)
            rrt_trajectory = self.rrt.create_rrt_trajectory(target2,
                                                            self.collisions)
            if rrt_trajectory.size == 0:
                self.trajectory = None
                self.rrt.graph.setJointState(start)
                self.rrt.graph.calcBodyFramesFromJoints()
                self.rrt.graph.calcBodyFramesFromJoints()
                raise TrajectoryException()
            std_traj = util.shorten_trajectory(rrt_trajectory, 250)
            self.trajectory = self.rrt.optimize_trajectory(std_traj,
                                                           collisions=
                                                           self.collisions,
                                                           endeff=self.endeff,
                                                           goal=self.goal.pos)
            self.trajectory = rrt_trajectory

        self.tpos = 0
        self.recompute_trajectory = False
        print start

        rospy.loginfo("done computing trajectory")

    def step(self):
        #print "step"
        if self.recompute_trajectory and self.goal:
            try:
                self.compute_trajectory()
            except TrajectoryException:
                rospy.loginfo("abort computing trajectory")
                self.control_done(success=False)

        if self.trajectory is not None:
            # rospy.loginfo("Move one step")
            if self.tpos == self.trajectory.shape[0]:
                self.control_done()
            else:
                rospy.logdebug("next step: " +
                               str(self.trajectory[self.tpos, :]))
                self.world.graph.setJointState(self.trajectory[self.tpos, :])
                self.world.graph.calcBodyFramesFromJoints()
                self.world.graph.calcBodyFramesFromJoints()
                self.tpos += 1

        self.physx.step()
        self.world.graph.calcBodyFramesFromJoints()
        self.world.graph.calcBodyFramesFromJoints()
        self.gl.update()

    def control_done(self, success=True):
        self.trajectory = None
        msg = msgs.control_done()
        msg.header.frame_id = 'control done'
        msg.success = success
        self.control_done_pub.publish(msg)
        self.goal = None

    def pstep(self):
        # P-Controller
        Kp = 10e-2
        # tolerance for he movement
        eps = 10e-2
        agent = self.world.graph.getBodyByName("robot")

        if self.goal:
            if (agent.X.pos - self.goal.pos).length() > eps:

                agent.X.pos = (agent.X.pos +
                               (self.goal.pos - agent.X.pos) * Kp)

            else:
                self.control_done()

        self.physx.step()
        self.world.graph.calcBodyFramesFromJoints()
        self.gl.update()

    def control_cb(self, data):
        new_goal = core.Transformation()
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
        new_goal = core.Transformation()
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
    print "Controller starting..."
    controller = FakeController()
    controller.run()

import orspy as ors
import corepy as core
import guipy as gui
import motionpy as motion
import numpy as np
import random


class Controller:
    def __init__(self):
        self.graph = ors.Graph("world.ors")
        shapes = ors.makeConvexHulls(self.graph.shapes)
        self.graph.shapes = shapes
        self.gl = gui.OpenGL()
        ors.bindOrsToOpenGL(self.graph, self.gl)

    def create_endpose(self, start, col_prec, pos_prec):

        problem = motion.MotionProblem(self.graph)
        problem.loadTransitionParameters()
        problem.H_rate_diag = core.getArrayParameter("Hratediag")

        shapes = core.getIntAParameter("agent_shapes")

        proxy_tm = motion.ProxyTaskMap(motion.allVersusListedPTMT,
                                       shapes,
                                       .01,
                                       True)
        task_cost2 = problem.addTaskMap("proxyColls", proxy_tm)
        problem.setInterpolatingCosts(task_cost2,
                                      motion.MotionProblem.constant,
                                      np.array([0]), col_prec)

        position_tm = motion.DefaultTaskMap(motion.posTMT,
                                            self.graph,
                                            "tip1",
                                            core.Vector(0, 0, 0))
        task_cost = problem.addTaskMap("position", position_tm)
        problem.setInterpolatingCosts(task_cost,
                                      motion.MotionProblem.finalOnly,
                                      core.ARRAY(problem.ors
                                                 .getBodyByName("target")
                                                 .X.pos),
                                      pos_prec)
        problem.setInterpolatingVelCosts(task_cost, motion
                                         .MotionProblem.finalOnly,
                                         np.array([0., 0., 0.]), 1e1)

        _, x = motion.keyframeOptimizer(start, problem, True, 2)
        return x

    def create_rrt_trajectory(self, target):
        stepsize = .005

        problem = motion.MotionProblem(self.graph)
        problem.loadTransitionParameters()
        
        shapes = core.getIntAParameter("agent_shapes")
        proxy_tm = motion.ProxyTaskMap(motion.allVersusListedPTMT, shapes, .01,
                                       True)
        task_cost = problem.addTaskMap("proxyColls", proxy_tm)

        problem.setInterpolatingCosts(task_cost,
                                      motion.MotionProblem.constant,
                                      np.array([0]), 1e-0)
        task_cost.y_threshold = 0

        planner = motion.RRTPlanner(self.graph, problem, stepsize)
        planner.joint_max = core.getArrayParameter("joint_max")
        planner.joint_min = core.getArrayParameter("joint_min")

        traj = planner.getTrajectoryTo(target)
        return traj

    def show_trajectory(self, trajectory):
        for pos in trajectory:
            self.graph.setJointState(pos)
            self.graph.calcBodyFramesFromJoints()
            self.gl.update()


def main():
    con = Controller()

    start = con.graph.getJointState()
    opt_start = con.graph.getJointState()
    opt_start[0] = con.graph.getBodyByName("target").X.pos.x
    opt_start[1] = con.graph.getBodyByName("target").X.pos.y

    print("q = " + str(opt_start))

    target = con.create_endpose(opt_start, 1e0, 1e3)
    target2 = con.create_endpose(target, 1e3, 0)
    print("target = " + str(target2))

    con.graph.setJointState(start)

    traj = con.create_rrt_trajectory(target2)
    print(traj)

    con.show_trajectory(traj)


if __name__ == '__main__':
    main()

import orspy as ors
import corepy as core
import guipy as gui
import motionpy as motion
import numpy as np


class Controller:
    def __init__(self):
        self.graph = ors.Graph("world.ors")
        shapes = ors.makeConvexHulls(self.graph.shapes)
        self.graph.shapes = shapes
        self.gl = gui.OpenGL()
        ors.bindOrsToOpenGL(self.graph, self.gl)

    def create_endpose(self):

        problem = motion.MotionProblem(self.graph)
        problem.loadTransitionParameters()
        problem.H_rate_diag = motion.pr2_reasonable_W()

        shapes = motion.pr2_get_shapes(self.graph)

        proxy_tm = motion.ProxyTaskMap(motion.allVersusListedPTMT, shapes,
                                       .01, True)
        task_cost2 = problem.addTaskMap("proxyColls", proxy_tm)
        problem.setInterpolatingCosts(task_cost2,
                                      motion.MotionProblem.constant,
                                      np.array([0]), 1e-0)

        position_tm = motion.DefaultTaskMap(motion.posTMT, self.graph, "tip1",
                                            core.Vector(0, 0, 0))
        task_cost = problem.addTaskMap("position", position_tm)
        problem.setInterpolatingCosts(task_cost,
                                      motion.MotionProblem.finalOnly,
                                      core.ARRAY(problem.ors
                                                 .getBodyByName("target")
                                                 .X.pos),
                                      1e2)
        problem.setInterpolatingVelCosts(task_cost, motion
                                         .MotionProblem.finalOnly,
                                         np.array([0., 0., 0.]), 1e1)

        _, x = motion.keyframeOptimizer(problem.x0, problem, False, 2)
        return x

    def create_rrt_trajectory(self, target):
        stepsize = .005

        problem = motion.MotionProblem(self.graph)
        problem.loadTransitionParameters()

        shapes = motion.pr2_get_shapes(self.graph)
        proxy_tm = motion.ProxyTaskMap(motion.allVersusListedPTMT, shapes, .01, 
                                       True)
        task_cost = problem.addTaskMap("proxyColls", proxy_tm)

        problem.setInterpolatingCosts(task_cost,
                                      motion.MotionProblem.constant,
                                      np.array([0]), 1e-0)
        task_cost.y_threshold = 0

        planner = motion.RRTPlanner(self.graph, problem, stepsize)
        q = np.array([.999998, .500003, .999998, 1.5, -2, 0, .500003])
        planner.joint_max = q + np.ones(q.shape)
        planner.joint_min = q - np.ones(q.shape)

        traj = planner.getTrajectoryTo(target)
        return traj


def main():
    con = Controller()

    start = con.graph.getJointState()
    print("q = " + str(start))

    target = con.create_endpose()
    con.graph.setJointState(start)
    print("target = " + str(target))

    traj = con.create_rrt_trajectory(target)
    print(traj)


if __name__ == '__main__':
    main()

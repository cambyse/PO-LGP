import time
import numpy as np
from task_map import TaskMap
from komo import PyKOMO
import matplotlib.pyplot as plt

class TargetPosition(TaskMap):
    def __init__(self, goal=np.array([0, 0])):
        super(TargetPosition, self).__init__(name="target_position", order=0, dim=2)
        self.goal = goal
    def phi(self, x):
        cost = x - self.goal
        Jcost = np.array([[1, 0], [0, 1]])
        return cost, Jcost

class TargetVelocity(TaskMap):
    def __init__(self, goal=np.array([0, 0])):
        super(TargetVelocity, self).__init__(name="target_velocity", order=1, dim=2)
        self.goal = goal
    def phi(self, v):
        v_cost = v - self.goal
        Jv_cost = np.array([[1, 0], [0, 1]])
        return v_cost, Jv_cost

class AccelerationPenalty(TaskMap):
    def __init__(self):
        super(AccelerationPenalty, self).__init__(name="acceleration_penalty", order=2, dim=2)
    def phi(self, a):
        a_cost = np.array([a[0], a[1]])
        Ja_cost = np.array([[1, 0], [0, 1]])
        return a_cost, Ja_cost

if __name__ == "__main__":
    x0 = np.array([0, 0])
    print("x0:{}".format(x0))

    n_steps = 11
    end_common_root = 3
    end_branch_1 = n_steps-4
    p_1 = 0.4
    path_1 = [(i, p_1) for i in range(end_common_root)] + [(i, p_1) for i in range(end_common_root, end_branch_1)]
    path_2 = [(i, 1 - p_1) for i in range(end_common_root)] + [(i, 1 - p_1) for i in range(end_branch_1, n_steps)]

    komo = PyKOMO()
    komo.set_n_steps(n_steps)

    komo.add_task(TargetPosition(goal=[0, 0]), start=0, end=1)

    komo.add_task(AccelerationPenalty(), wpath=path_1)
    komo.add_task(TargetPosition(goal=[3, 3]), wpath=path_1, start=4, end=-1)

    komo.add_task(AccelerationPenalty(), wpath=path_2)
    komo.add_task(TargetPosition(goal=[3, -3]), wpath=path_2, start=4, end=-1)

    #komo.add_task(TargetPosition(goal=[5, 0]), start=5, end=6)
    #komo.add_task(TargetVelocity(goal=[1, 2]), path=linear_path, start=0, end=3)
    #komo.add_task(TargetVelocity(goal=[1, -2]), path=linear_path, start=3, end=6)
    #komo.add_task(TargetVelocity(goal=[1, 0]), path=linear_path, start=8, end=-1)

    start = time.time()
    x = komo.run(x0)
    end = time.time()

    print("final x:{}\ncost(x):{}\nopt-time:{}".format(x, komo.traj_cost(x), end - start))

    # plot traj
    colors = [1.0 - i / 2 / len(x) for i in range(len(x))]
    plt.axis('equal')
    plt.scatter(x[:,0], x[:,1], c=colors)
    plt.show()

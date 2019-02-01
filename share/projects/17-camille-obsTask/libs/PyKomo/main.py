import time
import matplotlib.pyplot as plt

from task_map import *
from komo import PyKOMO
from path_builder import PathBuilder

if __name__ == "__main__":
    x0 = np.array([0, 0])
    print("x0:{}".format(x0))

    # n_phases = 11
    # end_common_root = 3
    # end_branch_1 = n_phases-4
    # p_1 = 0.5
    # path_1 = [(i, p_1) for i in range(end_common_root)] + [(i, p_1) for i in range(end_common_root, end_branch_1)]# + [(0, 0.5), (1, 0.5)]
    # path_2 = [(i, 1 - p_1) for i in range(end_common_root)] + [(i, 1 - p_1) for i in range(end_branch_1, n_phases)]# + [(0, 0.5), (1, 0.5)]
    p_1 = 0.5

    pb = PathBuilder()
    pb.add_edge(0, 1)
    pb.add_edge(1, 2)
    pb.add_edge(2, 3, p_1)
    pb.add_edge(3, 4, p_1)
    pb.add_edge(4, 5, p_1)
    pb.add_edge(5, 6, p_1)

    pb.add_edge(2, 7, 1 - p_1)
    pb.add_edge(7, 8, 1 - p_1)
    pb.add_edge(8, 9, 1 - p_1)
    pb.add_edge(9, 10, 1 - p_1)

    path_1, path_2 = pb.get_paths()

    komo = PyKOMO()
    komo.set_n_phases(pb.n_nodes())

    komo.add_task(TargetPosition(goal=[0, 0]), start=0, end=1)


    # pos
    komo.add_task(AccelerationPenalty(), wpath=path_1)
    komo.add_task(TargetPosition(goal=[3, 3]), wpath=path_1, start=4, end=-1)

    komo.add_task(AccelerationPenalty(), wpath=path_2)
    komo.add_task(TargetPosition(goal=[3, -3]), wpath=path_2, start=4, end=-1)

    # vel
    # komo.add_task(AccelerationPenalty(), wpath=path_1)
    # komo.add_task(TargetVelocity(goal=[1, 1]), wpath=path_1, start=4, end=-1)
    #
    # komo.add_task(AccelerationPenalty(), wpath=path_2)
    # komo.add_task(TargetVelocity(goal=[1, -1]), wpath=path_2, start=4, end=-1)

    # pos
    # komo.add_task(AccelerationPenalty(), wpath=path_1)
    # komo.add_task(TargetPosition(goal=[3, 3]), wpath=path_1, start=3, end=4)
    #
    # komo.add_task(AccelerationPenalty(), wpath=path_2)
    # komo.add_task(TargetPosition(goal=[3, -3]), wpath=path_2, start=3, end=4)

    start = time.time()
    x = komo.run(x0)
    end = time.time()

    print("final x:{}\ncost(x):{}\nopt-time:{}".format(x, komo.traj_cost(x), end - start))

    # plot traj
    colors = [1.0 - i / 2 / len(x) for i in range(len(x))]
    plt.axis('equal')
    plt.scatter(x[:,0], x[:,1], c=colors)
    plt.show()

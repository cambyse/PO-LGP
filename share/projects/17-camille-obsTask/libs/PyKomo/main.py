import numpy as np
from task_map import TaskMap
from komo import PyKOMO

x_desired = 2
v_desired = 1
acc_coef = 0

class TargetPosition(TaskMap):
    def __init__(self):
        super(TargetPosition, self).__init__(name="target_position", order=0, dim=2)
    def phi(self, x):
        cost = np.array([x[0]-5, x[1]-1])
        Jcost = np.array([[1, 0], [0, 1]])
        return cost, Jcost

class TargetPositionX(TaskMap):
    def __init__(self):
        super(TargetPositionX, self).__init__(name="target_position x", order=0, dim=1)
    def phi(self, x):
        cost = np.array([x[0]-3])#([x[0]-x_desired, x[1]-1])
        Jcost = np.array([[1, 0]])
        return cost, Jcost

class TargetVelocity(TaskMap):
    def __init__(self):
        super(TargetVelocity, self).__init__(name="target_velocity", order=1)
    def phi(self, v):
        v_cost = np.array([v[0] - v_desired])
        Jv_cost = np.zeros(v.shape[0])
        Jv_cost = np.array([[1, 0]])
        return v_cost, Jv_cost

class AccelerationPenalty(TaskMap):
    def __init__(self):
        super(AccelerationPenalty, self).__init__(name="acceleration_penalty", order=2, dim=2)
    def phi(self, a):
        a_cost = np.array([a[0], a[1]])
        Ja_cost = np.array([[1, 0], [0, 1]])
        return a_cost, Ja_cost

if __name__ == "__main__":
    x0 = np.array([1, 0])
    print("x0:{}".format(x0))

    acc_phi = AccelerationPenalty()
    target_vel = TargetVelocity()
    target_pose = TargetPosition()
    #target_pose = TargetPositionX()

    komo = PyKOMO()
    komo.add_task(acc_phi)
    #komo.add_task(target_vel)
    komo.add_task(target_pose)
    komo.set_n_steps(10)
    x = komo.run(x0)

    print("final x:{}\ncost(x):{}".format(x, komo.traj_cost(x)))


import numpy as np

# from itertools import tee
#
# def pairwise(iterable):
#     a, b = tee(iterable)
#     next(b, None)
#     return zip(a, b)
#
# def triplewise(iterable):
#     a, b, c = tee(iterable, 3)
#     next(b, None)
#     next(c, None)
#     next(c, None)
#     return zip(a, b, c)

class TaskMap:
    def __init__(self, name = '', order = 0, dim = 1):
        self.order = order
        self.name = name
        self.dim = dim

    def phi(self, x):
        pass

    def phi(self, v, x):
        pass

class TargetPosition(TaskMap):
    def __init__(self, goal=np.array([0, 0])):
        super(TargetPosition, self).__init__(name="target_position", order=0, dim=2)
        self.goal = goal
    def phi(self, x):
        cost = x[:2] - self.goal
        Jcost = np.array([[1, 0, 0], [0, 1, 0]])
        return cost, Jcost

class TargetVelocity(TaskMap):
    def __init__(self, goal=np.array([0, 0])):
        super(TargetVelocity, self).__init__(name="target_velocity", order=1, dim=2)
        self.goal = goal
    def phi(self, v):
        v_cost = v[:2] - self.goal
        Jv_cost = np.array([[1, 0, 0], [0, 1, 0]])
        return v_cost, Jv_cost

class AccelerationPenalty(TaskMap):
    def __init__(self):
        super(AccelerationPenalty, self).__init__(name="acceleration_penalty", order=2, dim=2)
    def phi(self, a):
        a_cost = np.array([a[0], a[1]])
        Ja_cost = np.array([[1, 0, 0], [0, 1, 0]])
        return a_cost, Ja_cost

# class CarOrientation(TaskMap):
#     def __init__(self):
#         super(CarOrientation, self).__init__(name="car_orientation", order=0, dim=1)
#     def phi(self, a, x):
#         a_cost = np.array([a[0], a[1]])
#         Ja_cost = np.array([[1, 0, 0], [0, 1, 0]])
#         return a_cost, Ja_cost
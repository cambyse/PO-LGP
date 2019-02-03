import numpy as np
import math
from enum import Enum

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

class TaskMapType(Enum):
    COST = 1
    EQ = 2

class TaskMap:
    def __init__(self, name = '', order = 0, dim = 1, type = TaskMapType.COST):
        self.order = order
        self.name = name
        self.dim = dim
        self.type = type

    def phi(self, _, context):
        pass

class TargetPosition(TaskMap):
    def __init__(self, goal=np.array([0, 0])):
        super(TargetPosition, self).__init__(name="target_position", order=0, dim=2)
        self.goal = goal
    def phi(self, x, context):
        cost = x[:2] - self.goal
        Jcost = np.array([[1, 0, 0], [0, 1, 0]])
        return cost, Jcost

class TargetVelocity(TaskMap):
    def __init__(self, goal=np.array([0, 0])):
        super(TargetVelocity, self).__init__(name="target_velocity", order=1, dim=2)
        self.goal = goal
    def phi(self, v, context):
        v_cost = v[:2] - self.goal
        Jv_cost = np.array([[1, 0, 0], [0, 1, 0]])
        return v_cost, Jv_cost

class AccelerationPenalty(TaskMap):
    def __init__(self):
        super(AccelerationPenalty, self).__init__(name="acceleration_penalty", order=2, dim=2)
    def phi(self, a, context):
        a_cost = np.array([a[0], a[1]])
        Ja_cost = np.array([[1, 0, 0], [0, 1, 0]])
        return a_cost, Ja_cost

class CarOrientation(TaskMap):
    def __init__(self):
        super(CarOrientation, self).__init__(name="car_orientation", order=0, dim=2, type=TaskMapType.EQ)
    def phi(self, x, context):
        if context[0] is None:
            cost = np.array([0, 0])
            J_cost = np.array([[0, 0, 0], [0, 0, 0]]) #np.array([[0, 0, -math.sin(x[2])], [0, 0, math.cos(x[2])]])
        else:
            v = (context[1]-context[0])[0:2]
            normV = np.linalg.norm(v)
            if normV == 0:
                cost = np.array([0, 0])
                J_cost = np.array([[0, 0, 0], [0, 0, 0]])
            else:
                v /= normV
                theta = x[2]
                heading = np.array([math.cos(theta), math.sin(theta)])
                cost = heading - v
                J_cost = np.array([[0, 0, -math.sin(theta)], [0, 0, math.cos(theta)]])
        return cost, J_cost
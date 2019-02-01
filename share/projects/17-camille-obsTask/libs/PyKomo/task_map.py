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
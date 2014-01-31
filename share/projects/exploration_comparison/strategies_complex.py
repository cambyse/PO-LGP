import random
import math
import numpy as np
import scipy.stats as ss


class StrategyRandom(object):
    """Select objects randomly."""
    def __init__(self):
        self.name = "Random"

    def __call__(self, belief):
        return random.randint(0, len(belief) - 1)


class StrategyRoundRobin(object):
    """Select objects in a round robin fashion."""
    def __init__(self):
        self.name = "Round Robin"
        self.last_selection = -1

    def __call__(self, belief):
        self.last_selection = (self.last_selection + 1) % len(belief)
        return self.last_selection


class StrategyMaxEntropyObjType(object):
    """Select the object with the highest entropy."""
    def __init__(self):
        self.name = "Max Entropy for the obj type"

    def __call__(self, belief):
        return max_idx(belief, f=(lambda obj: obj.entropy()))


class StrategyMaxEntropyJointType(object):
    """Select the object with the highest entropy."""
    def __init__(self):
        self.name = "Max Entropy for the joint type"

    def __call__(self, belief):
        return max_idx(belief, f=(lambda obj: obj.joint_bel.entropy()))


class StrategyExpectedChangeOfEntropy(object):
    """Select the object with the highest expected change of entropy."""

    def __call__(self, belief):
        # collect change of entropy
        l = []
        for i, obj_bel in enumerate(belief):
            l.append((i, obj_bel.entropy_diff()))
            l.append((i, obj_bel.joint_bel.entropy_diff()))

            change, stats = obj_bel.joint_bel.entropy_tmp()
            for key, val in change.iteritems():
                l.append((i, val))

        # select change of entropy
        idx, val = max(l, key=lambda tuple_: tuple_[1])
        return idx


def max_idx(iterable, f):
    """Return the index where f(elem) is max"""
    current_max = -99999
    for i, obj in enumerate(iterable):
        val = f(obj)
        if val > current_max:
            current_max, idx = val, i
        # print("looking at {}: {} current_max {} with {}".format(
        #     i, val, idx, current_max))
    return idx

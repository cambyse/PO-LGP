import numpy as np
import math


###############################################################################
# dehomogenize a homogeneous vector
def dehomogenize(w):
    return (np.array(w) / w[3])[:3].tolist()


###############################################################################
#Return homogeneous rotation matrix from quaternion.
def quaternion_matrix(quaternion):

    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])


###############################################################################
# generic datastructure manipulation
def flatten(iterable):
    """Given an iterable, possibly nested to any level, retustriprn it flattened.

    >>> nested_list = [1, [2, 2], [3, 3, 3, [4, 4, 4, 4,], 3], 1]
    >>> flatten(nested_list)
    [1, 2, 2, 3, 3, 3, 4, 4, 4, 4, 3, 1]

    >>> nested_list = ["one", ["two", "three", ["four"]]]
    >>> flatten(nested_list)
    ['one', 'two', 'three', 'four']

    """
    new_list = []
    for item in iterable:
        if hasattr(item, '__iter__'):
            new_list.extend(flatten(item))
        else:
            new_list.append(item)
    return new_list


###############################################################################
# Sides
class SIDE:
    LEFT = "left"
    RIGHT = "right"
    DEFAULT = "right"

class Bg_facts:
    _facts = {}

    @property
    def facts(self):
        return Bg_facts._facts

    @facts.setter
    def facts(self,facts):
        Bg_facts._facts.update(facts)


###############################################################################
def assert_in(name, source):
    """Assert that ``name`` is in ``source``."""
    if name not in source:
        raise ValueError("{} is not a valid string".format(name))


###############################################################################
# Helper: symbols...
def strip_specs(fact):
    """Remove the specifications/parameters from the given symbol and return
    it.

    >>> fact = "(fact1 fact2){ value=1 pos=[2, 2, 2]}"
    >>> strip_specs(fact)
    '(fact1 fact2)'

    """
    return fact[: fact.find(")")] + ")"


def conv_symbol(symbol):
    """
    Return 'converged' symbol of the given symbol by prepending `conv`.

    >>> conv_symbol("(action)")
    '(conv action)'

    """
    return '(conv ' + symbol[1:]


def pos_str2arr(pos_str):
    """
    Turn string pos into a numpy array.

    >>> pos_str2arr("[(3 4 5)]")
    array([ 3.,  4.,  5.])

    """
    return np.array([float(i) for i in pos_str[2:-2].split(" ")])


def side2endeff(side=None):
    """
    Use the SIDE.DEFAULT if side is None.

    >>> SIDE.DEFAULT = SIDE.RIGHT
    >>> side2endeff()
    'endeffR'
    """
    source = {SIDE.LEFT: "endeffL", SIDE.RIGHT: "endeffR"}
    return source.get(side, source[SIDE.DEFAULT])


def side2gripper_joint(side=None):
    """
    Use the SIDE.DEFAULT if side is None.

    >>> SIDE.DEFAULT = SIDE.RIGHT
    >>> side2gripper_joint()
    'r_gripper_joint'
    """
    source = {SIDE.LEFT: "l_gripper_joint", SIDE.RIGHT: "r_gripper_joint"}
    return source.get(side, source[SIDE.DEFAULT])

def side2wrist_joint(side=None):
    """
    Use the SIDE.DEFAULT if side is None.

    >>> SIDE.DEFAULT = SIDE.RIGHT
    >>> side2wrist_joint()
    'r_wrist_roll_joint'
    """
    source = {SIDE.LEFT: "l_wrist_roll_joint",
              SIDE.RIGHT: "r_wrist_roll_joint"}
    return source.get(side, source[SIDE.DEFAULT])

def setFixBase(base = True):
    """
    If True, the base is fixed.
    If False, the robot is movable
    One can set it in the MT.cfg, too.
    """
    interface.setFixBase(base)

def setVerbose(verbose = True):
    """
    If true, bump state every tick.
    If false, not.
    """
    interface.setVerbose(verbose)



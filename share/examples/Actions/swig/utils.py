import numpy as np


###############################################################################
# generic datastructure manipulation
def flatten(iterable):
    """Given an iterable, possibly nested to any level, return it flattened.

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
    DEFAULT = "left"


###############################################################################
def assert_valid_shapes(name, shapes):
    if name not in shapes:
        raise ValueError("The given shape {} is not an existing shape"
                         .format(name))


def assert_valid_joints(name, joints):
    if name not in joints:
        raise ValueError("The given joint {} is not an existing joint"
                         .format(name))


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

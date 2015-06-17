from __future__ import print_function
from collections import namedtuple
import time

import swig


# interface.setFact("(HomingActivity)")
# interface.waitForCondition("(conv HomingActivity)")
# interface.setFact("(conv HomingActivity)!")
# interface.setFact("(GripperOpen){ ref1=l_gripper_joint }")
# interface.waitForCondition("(conv GripperOpen)")
# interface.setFact("(conv GripperOpen)!")

time.sleep(.2)
interface = swig.ActionSwigInterface(1)
time.sleep(.2)

DEFAULT_ENDEFFECTOR = "endeffL"
print("=" * 70)
print("DEFAULT_ENDEFFECTOR is set to {}".format(DEFAULT_ENDEFFECTOR))
print("  You can always change it: `DEFAULT_ENDEFFECTOR = endffR`")

# Sides
LEFT = 256
RIGHT = 255


###############################################################################
# Helper functions
def shapes(name=None):
    return interface.getShapeByName(name) if name else interface.getShapeList()


def joints(name=None):
    return interface.getJointByName(name) if name else interface.getJointList()


def bodies(name=None):
    return interface.getBodyByName(name) if name else interface.getBodyList()


def facts():
    return interface.getFacts()


###############################################################################
# Convenient access and autocompletion to shapes, joints, and bodies
# Just type `s.<tab>` to get a list of all shapes
Shapes = namedtuple("Shapes", " ".join(shapes()))
s = Shapes(*shapes())


Bodies = namedtuple("Bodies", " ".join(bodies()))
b = Bodies(*bodies())


Joints = namedtuple("Joints", " ".join(joints()))
j = Joints(*joints())


###############################################################################
def _run(fact):
    """Run the given fact, wait for its completion and remove the fact
    afterwards.
    """
    symbols = fact[: fact.find(")")] + ")"
    symbols_conv = '(conv ' + symbols[1:]

    interface.setFact(fact)
    interface.waitForCondition(symbols_conv)
    interface.stopFact(symbols)
    interface.stopFact(symbols_conv)


def assert_valid_shapes(shape):
    if shape not in shapes():
        raise ValueError("The given shape {} is not an existing shape"
                         .format(shape))


###############################################################################
# Python activities
def homing():
    _run("(HomingActivity)")


def _gripper(side, target):
    if side == LEFT:
        endeff, joint = "endeffL", "l_gripper_joint"
    elif side == RIGHT:
        endeff, joint = "endeffR", "r_gripper_joint"
    else:
        raise ValueError("side should be LEFT or RIGHT")
    assert_valid_shapes(target)

    fact = "(GripperActivity %s){ ref1=%s, target=[%f] tol=.02 }" % (
        endeff, joint, target)
    _run(fact)


def open_gripper(side):
    _gripper(side, .08)


def close_gripper(side):
    _gripper(side, .01)


def touch(what, with_=None, offset=None):
    """bla"""
    if with_ is None:
        with_ = DEFAULT_ENDEFFECTOR
    if offset is None:
        offset = (0., 0., 0.)
    offset = "[{} {} {}]".format(*offset)
    assert_valid_shapes(what)
    assert_valid_shapes(with_)

    _run(
        "(FollowReferenceActivity {ref1} {ref2}){{ type=pos, ref1={ref1}, ref2={ref2} target={offset} }}"
         .format(ref1=with_, ref2=what, offset=offset))


if __name__ == '__main__':
    pass

    # open_gripper(LEFT)
    # close_gripper(LEFT)

    # open_gripper(RIGHT)
    # close_gripper(RIGHT)

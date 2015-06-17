from __future__ import print_function
from collections import namedtuple
import swig


# s.setFact("(HomingActivity)")
# s.waitForCondition("(conv HomingActivity)")
# s.setFact("(conv HomingActivity)!")
# s.setFact("(GripperOpen){ ref1=l_gripper_joint }")
# s.waitForCondition("(conv GripperOpen)")
# s.setFact("(conv GripperOpen)!")

interface = swig.ActionSwigInterface(1)
print(dir(interface))


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
    return s.getFacts()


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

    s.setFact(fact)
    s.waitForCondition(symbols_conv)
    s.stopFact(symbols)
    s.stopFact(symbols_conv)


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

    fact = "(GripperActivity %s){ ref1=%s, target=[%f] tol=.02 }" % (
        endeff, joint, target)
    _run(fact)


def open_gripper(side):
    _gripper(side, .08)


def close_gripper(side):
    _gripper(side, .01)


def touch(what, with_, offset=None):
    """bla"""
    if offset is None:
        offset = (0., 0., 0.)
    offset = "[{} {} {}]".format(*offset)
    _run(
        "(FollowReferenceActivity {ref1} {ref2}){{ type=pos, ref1={ref1}, ref2={ref2} target={offset} }}"
         .format(ref1=with_, ref2=what, offset=offset))


if __name__ == '__main__':
    pass

    # open_gripper(LEFT)
    # close_gripper(LEFT)

    # open_gripper(RIGHT)
    # close_gripper(RIGHT)

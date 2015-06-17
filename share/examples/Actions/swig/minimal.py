from __future__ import print_function
import swig


# s.setFact("(HomingActivity)")
# s.waitForCondition("(conv HomingActivity)")
# s.setFact("(conv HomingActivity)!")
# s.setFact("(GripperOpen){ ref1=l_gripper_joint }")
# s.waitForCondition("(conv GripperOpen)")
# s.setFact("(conv GripperOpen)!")

s = swig.ActionSwigInterface(1)
print(dir(s))


###############################################################################
# Helper functions
def shapes(name=None):
    return s.getShapeByName(name) if name else s.getShapeList()


def joints(name=None):
    return s.getJointByName(name) if name else s.getJointList()


def bodies(name=None):
    return s.getBodyByName(name) if name else s.getBodyList()


def facts():
    return s.getFacts()


###############################################################################
LEFT = 0
RIGHT = 1


def _run(fact):
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

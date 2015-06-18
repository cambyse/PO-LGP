"""
A simple and convenient python interface for the ActivityMachine

"""
from __future__ import print_function
from collections import namedtuple
import time
import signal

from contextlib import contextmanager
import swig


def signal_handler(signal, frame):
    print("\n"*10)
    print("\nYou pressed Ctrl+C!")
    print("Stopping all facts")
    new_facts = []
    for fact in facts():
        print("\n"*10)
        print(fact)
        if "conv" not in fact:
            new_facts.append("(conv " + fact[1: fact.find(")") + 1])

    for fact in new_facts:
        interface.setFact(fact)


interface = swig.ActionSwigInterface(1)

# don't abort the swig interface on Ctr-C
# the signal must be geristered after the swig interface was initialized
# signal.signal(signal.SIGINT, signal_handler)
time.sleep(.2)


###############################################################################
# Sides
LEFT = 256
RIGHT = 255

DEFAULT_SIDE = LEFT
eff = {LEFT: "endeffL", RIGHT: "endeffR"}

print("=" * 70)
print("DEFAULT_SIDE is set to {}".format(DEFAULT_SIDE))
print("  You can always change it: `DEFAULT_SIDE = RIGHT`")


###############################################################################
# Helper: access ors structures
def shapes(name=None):
    return interface.getShapeByName(name) if name else interface.getShapeList()


def joints(name=None):
    return interface.getJointByName(name) if name else interface.getJointList()


def bodies(name=None):
    return interface.getBodyByName(name) if name else interface.getBodyList()


def facts():
    return interface.getFacts()


###############################################################################
def assert_valid_shapes(shape):
    if shape not in shapes():
        raise ValueError("The given shape {} is not an existing shape"
                         .format(shape))

###############################################################################
# Convenient access and autocompletion to shapes, joints, and bodies
# Just type `s.<tab>` to get a list of all shapes
_tmp = list(shapes())
Shapes = namedtuple("Shapes", " ".join(_tmp))
s = Shapes(*_tmp)

_tmp = list(bodies())
Bodies = namedtuple("Bodies", " ".join(_tmp))
b = Bodies(*_tmp)

_tmp = list(joints())
Joints = namedtuple("Joints", " ".join(_tmp))
j = Joints(*_tmp)

del _tmp


###############################################################################
# Helper: symbols...
def strip_specs(fact):
    return fact[: fact.find(")")] + ")"


def conv_symbol(symbol):
    return '(conv ' + symbol[1:]


###############################################################################
# execution control
def run(facts):
    """Run the given fact, wait for its completion and remove the fact
    afterwards.
    """
    if not isinstance(facts, list):
        facts = [facts]

    symbols = [strip_specs(fact) for fact in facts]
    symbols_conv = [conv_symbol(symbol) for symbol in symbols]

    for fact in facts:
        interface.setFact(fact)

    for symb_conv in symbols_conv:
        interface.waitForCondition(symb_conv)

    for symb in symbols:
        interface.stopFact(symb)

    for symb_conv in symbols_conv:
        interface.stopFact(symb_conv)


@contextmanager
def running(facts):
    """
    Context manager to run the given facts (list of strings) during the context
    """
    for fact in facts:
        interface.setFact(fact)

    yield

    symbols = [strip_specs(fact) for fact in facts]
    symbols_conv = [conv_symbol(symbol) for symbol in symbols]

    for symb in symbols:
        interface.stopFact(symb)

    active_facts = interface.getFacts()
    for symb_conv in symbols_conv:
        if symb_conv + "," in active_facts:
            interface.stopFact(symb_conv)


###############################################################################
# Python activities
def homing():
    return ["(HomingActivity){ tol=.04 }"]


def _gripper(side, target):
    if side is None:
        side = LEFT
    if side == LEFT:
        endeff, joint = "endeffL", "l_gripper_joint"
    elif side == RIGHT:
        endeff, joint = "endeffR", "r_gripper_joint"
    else:
        raise ValueError("side should be LEFT or RIGHT")
    assert_valid_shapes(endeff)

    fact = ("(GripperActivity {endeff}){{ ref1={joint}, target=[{target}] tol=.01 }}"
            .format(endeff=endeff, joint=joint, target=target))
    return [fact]


def open_gripper(side=None):
    return _gripper(side, .08)


def close_gripper(side=None):
    return _gripper(side, .01)


def reach(what, with_=None, offset=None):
    """bla"""
    if with_ is None:
        with_ = eff[DEFAULT_SIDE]
    if offset is None:
        offset = (0., 0., 0.)

    offset = "[{} {} {}]".format(*offset)
    assert_valid_shapes(what)
    assert_valid_shapes(with_)

    fact = "(FollowReferenceActivity {ref1} {ref2}){{ type=pos, ref1={ref1}, ref2={ref2}, target={offset} }}".format(ref1=with_, ref2=what, offset=offset)
    return [fact]


def align_gripper_horizontal(side=None):
    return align_gripper([0, 0, 1], [0, 0, -1], side)


def align_gripper_vertical(side=None):
    return align_gripper([0, 1, 0], [0, 0, -1], side)


def align_gripper(vec_endeff, vec_target, side=None):
    if side is None:
        side = DEFAULT_SIDE
    endeff = eff[side]

    assert_valid_shapes(endeff)

    if " rot," not in interface.getSymbols():
        interface.createNewSymbol("rot")

    fact = "(FollowReferenceActivity {ref1} rot){{ type=vec, ref1={ref1}, vec1={vec_endeff}, target={vec_target} }}".format(ref1=endeff, vec_endeff=vec_endeff, vec_target=vec_target)

    return [fact]


def align_gripper_with_plane(front_opening, rotation_around_wrist, side=None):
    if side is None:
        side = DEFAULT_SIDE
    endeff = eff[side]

    assert_valid_shapes(endeff)

    if " rot," not in interface.getSymbols():
        interface.createNewSymbol("rot")
        interface.createNewSymbol("front")

    return [
        "(FollowReferenceActivity {ref1} front){{ type=vec, ref1={ref1}, vec1=[1 0 0], target={front} }}".format(ref1=endeff, front=front_opening),
        "(FollowReferenceActivity {ref1} rot){{ type=vec, ref1={ref1}, vec1=[0 1 0], target={rot} }}".format(ref1=endeff, rot=rotation_around_wrist)
    ]


def gaze_at(shape):
    if " gazeAt," not in interface.getSymbols():
        interface.createNewSymbol("gazeAt")

    assert_valid_shapes(shape)

    pos = "[" + shapes(shape)["pos"][2:-2] + "]"
    print(pos)
    fact = """
    (FollowReferenceActivity gazeAt){{
        type=gazeAt
        ref1=endeffHead ref2=base_footprint
        vec1=[0, 0, 1] vec2={pos} }}
    """.strip().format(pos=pos)
    return [fact]


###############################################################################
# High Level Behaviors
def grab_marker(shape):
    with running(gaze_at("endeffL")):
        run(open_gripper(LEFT)
            + reach(shape, offset=[0.0, 0.01, 0.1])
            + align_gripper_with_plane([1, 0, 0], [0, -1, 0]))
        run(reach(shape, offset=[0.0, 0.01, -0.07]))
        run(close_gripper(LEFT))
    # run(homing())


###############################################################################
if __name__ == '__main__':
    # run(homing())
    pass
    # open_gripper(RIGHT)
    # close_gripper(RIGHT)
    # grab_marker("mymarker")

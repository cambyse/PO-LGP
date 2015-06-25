from __future__ import print_function

from contextlib import contextmanager
import numpy as np

from resources import interface, shapes, s, bodies, b, joints, j
from utils import (
    SIDE,
    strip_specs,
    conv_symbol,
    assert_valid_shapes,
    side2endeff,
    side2gripper_joint,
    pos_str2arr,
    side2wrist_joint,
)


###############################################################################
# execution actions
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
    """Context manager to run the given facts (list of strings) during the
    context.

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
    endeff = side2endeff(side)
    joint = side2gripper_joint(side)

    assert_valid_shapes(endeff, shapes())

    fact = ("(FollowReferenceActivity {endeff} gripper)"
            "{{ type=qItself ref1={joint}, target=[{target}] tol=.01 }}"
            .format(endeff=endeff, joint=joint, target=target))
    return [fact]


def open_gripper(side=None):
    return _gripper(side, .08)


def close_gripper(side=None):
    return _gripper(side, .01)


def reach(what, with_=None, offset=None):
    """bla"""
    if with_ is None:
        with_ = side2endeff()
    if offset is None:
        offset = [0., 0., 0.]

    assert_valid_shapes(what, shapes())
    assert_valid_shapes(with_, shapes())

    # offset = "[{} {} {}]".format(*offset)
    fact = ("(FollowReferenceActivity {ref1} {ref2})"
            "{{ type=pos, ref1={ref1}, ref2={ref2}, target={offset} }}"
            .format(ref1=with_, ref2=what, offset=offset))
    return [fact]


def align_gripper_horizontal(side=None):
    return align_gripper([0, 0, 1], [0, 0, -1], side)


def align_gripper_vertical(side=None):
    return align_gripper([0, 1, 0], [0, 0, -1], side)


def align_gripper(vec_endeff, vec_target, side=None):
    endeff = side2endeff(side)

    assert_valid_shapes(endeff, shapes())

    fact = ("(FollowReferenceActivity {ref1} rot)"
            "{{ type=vec, ref1={ref1}, vec1={vec_endeff}, "
            "target={vec_target} }}"
            .format(ref1=endeff, vec_endeff=vec_endeff, vec_target=vec_target))

    return [fact]


def align_gripper_with_plane(front_opening, rotation_around_wrist, side=None):
    endeff = side2endeff(side)

    assert_valid_shapes(endeff, shapes())

    return [
        ("(FollowReferenceActivity {ref1} front)"
         "{{ type=vec, ref1={ref1}, vec1=[1 0 0], target={front} }}"
         .format(ref1=endeff, front=front_opening)),
        ("(FollowReferenceActivity {ref1} rot)"
         "{{ type=vec, ref1={ref1}, vec1=[0 1 0], target={rot} }}"
         .format(ref1=endeff, rot=rotation_around_wrist))
    ]


def gaze_at(shape):
    assert_valid_shapes(shape, shapes())

    pos = pos_str2arr(shapes(shape)["pos"])
    fact = """
    (FollowReferenceActivity gazeAt){{
        type=gazeAt
        ref1=endeffHead ref2=base_footprint
        vec1=[0, 0, 1] vec2={pos} }}
    """.strip().format(pos=pos)
    return [fact]


def move_along_axis(endeff, axis, distance):
    axis = np.asarray(axis)

    endeff_pos = pos_str2arr(interface.getShapeByName(endeff)["pos"])
    print(endeff_pos)

    target_pos = endeff_pos + distance/np.linalg.norm(axis) * axis
    print(target_pos)

    fact = ("(FollowReferenceActivity {ref1})"
            "{{ type=pos, ref1={ref1}, vec2={pos} }}"
            .format(ref1=endeff, pos=target_pos))
    return [fact]


def turn_wrist(rel_degree, side=None):
    joint = side2wrist_joint(side)
    current_q = float(joints(joint)["q"])
    target = current_q + np.deg2rad(rel_degree)
    fact = ("(FollowReferenceActivity qItself {joint})"
            "{{ type=qItself ref1={joint} target=[{target}] tol=.1 }}"
            .format(joint=joint, target=target))
    return [fact]


def move_to_pos(endeff, pos):
    fact = ("(FollowReferenceActivity pos {endeff})"
            "{{ type=pos ref1={endeff} vec2={position} }}"
            .format(endeff=endeff, position=pos))
    return [fact]


###############################################################################
# High Level Behaviors
def run_grab_marker(shape, side=None):
    endeff = side2endeff(side)
    with running(gaze_at(endeff)):
        run(open_gripper(side=side)
            + reach(shape, offset=[0.0, 0.01, 0.1], with_=endeff)
            + align_gripper_with_plane([1, 0, 0], [0, -1, 0], side=side))
        run(reach(shape, offset=[0.0, 0.01, -0.07], with_=endeff))
        run(close_gripper(side=side))
    # run(homing())


def run_turn_marker(shape, degree, side=None):
    endeff = side2endeff(side)
    with running(gaze_at(endeff)):
        run(open_gripper()
            + reach(shape, with_=endeff, offset=[0.0, 0.01, 0.1])
            + align_gripper_with_plane([1, 0, 0], [0, -1, 0], side=side))
        run(reach(shape, with_=endeff, offset=[0.0, 0.01, -0.07]))
        run(close_gripper(side))
        run(turn_wrist(degree, side))
        run(open_gripper(side))


def run_move_shape(shape, distance, side=None):
    endeff = side2endeff(side)
    run(align_gripper_with_plane([1, 0, 0], [0, -1, 0], side=side))
    with running(align_gripper_with_plane([1, 0, 0], [0, -1, 0], side=side) +
                 gaze_at(endeff)):
        run(open_gripper(side=side)
            + reach(shape, offset=[-0.05, 0, 0.0], with_=endeff))
        run(reach(shape, offset=[0.0, .0, 0.], with_=endeff))
        run(close_gripper(side=side))
        run(move_along_axis(endeff, [0, 0, 1], distance))
        run(open_gripper(side=side))


def run_move_shape_along_joint(shape, distance, joint, side=None):
    endeff = side2endeff(side)
    axis = pos_str2arr(interface.getJointByName(joint)["axis"])
    run(align_gripper_with_plane([1, 0, 0], [0, -1, 0], side=side))
    with running(align_gripper_with_plane([1, 0, 0], [0, -1, 0], side=side) +
                 gaze_at(endeff)):
        run(open_gripper(side=side)
            + reach(shape, offset=[-0.05, 0, 0.0], with_=endeff))
        run(reach(shape, offset=[0.0, .0, 0.], with_=endeff))
        run(close_gripper(side=side))
        run(move_along_axis(endeff, axis, distance))
        run(open_gripper(side=side))


print("Loaded actions.py...")

from __future__ import print_function

from contextlib import contextmanager
import numpy as np

from resources import interface, shapes, s, bodies, b, joints, j
from utils import (
    SIDE,
    strip_specs,
    conv_symbol,
    assert_valid_shapes,
    assert_valid_joints,
    side2endeff,
    side2gripper_joint,
    pos_str2arr,
    side2wrist_joint,
)


###############################################################################
# execution actions
def _run(facts):
    """Run the given fact, wait for its completion and remove the fact
    afterwards.

    """
    symbols = [strip_specs(str(fact)) for fact in facts]
    symbols_conv = [conv_symbol(symbol) for symbol in symbols]

    for fact in facts:
        interface.setFact(str(fact))

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
    if not isinstance(facts, list):
        facts = [facts]

    facts = _flatten(facts)

    for fact in facts:
        interface.setFact(str(fact))

    yield

    symbols = [strip_specs(str(fact)) for fact in facts]
    symbols_conv = [conv_symbol(symbol) for symbol in symbols]

    for symb in symbols:
        interface.stopFact(symb)

    active_facts = interface.getFacts()
    for symb_conv in symbols_conv:
        if symb_conv + "," in active_facts:
            interface.stopFact(symb_conv)


def _run_with(with_construct):
    with running(with_construct["with"]):
        run(with_construct["plan"])


def _flatten(iterable):
    """Given an iterable, possibly nested to any level, return it flattened."""
    new_list = []
    for item in iterable:
        if hasattr(item, '__iter__'):
            new_list.extend(_flatten(item))
        else:
            new_list.append(item)
    return new_list


def run(plan):
    if not isinstance(plan, list):
        plan = [plan]
    for item in plan:
        if isinstance(item, list):
            run(item)
        elif isinstance(item, dict):
            _run_with(item)
        elif isinstance(item, tuple):
            _run(_flatten(item))
        else:
            _run([item])

###############################################################################
# Python activitiy classes

class Activity(object):
    def __init__(self):
        self.time = 3.
        self.damping = .7
        self.max_vel = 10
        self.max_acc = 10
        self.tolerance = .01
        self._name = ""

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, name):
        if name + "," not in interface.getSymbols():
            interface.createNewSymbol(name)
        self._name = name

    @property
    def natural_gains(self):
        return [self.time, self.damping, self.max_vel, self.max_acc]

    @natural_gains.setter
    def natural_gains(self, gains):
        self.time, self.damping, self.max_vel, self.max_acc = gains

    def __repr__(self):
        return str(self)


class PosActivity(Activity):
    def __init__(self, endeff, pos):
        super(PosActivity, self).__init__()
        assert_valid_shapes(endeff, shapes())
        self.endeff = endeff
        self.pos = pos

    def __str__(self):
        return ("(FollowReferenceActivity {endeff} pos {name})"
                "{{ type=pos ref1={endeff} vec2={pos} tol={tol} PD={gains} }}"
                .format(name=self.name, endeff=self.endeff, pos=self.pos,
                        tol=self.tolerance, gains=self.natural_gains))


class MoveAlongAxisActivity(Activity):
    def __init__(self, endeff, axis, distance):
        super(MoveAlongAxisActivity, self).__init__()
        self.endeff = endeff
        self.axis = axis
        self.distance = distance

    def __str__(self):
        start_pos = np.array(pos_str2arr(interface.getShapeByName(self.endeff)["pos"]))
        target_pos = (start_pos +
                      self.distance/np.linalg.norm(self.axis) * self.axis)

        return ("(FollowReferenceActivity {endeff} pos {name})"
                "{{ type=pos ref1={endeff} vec2={pos} tol={tol} PD={gains} }}"
                .format(name=self.name, endeff=self.endeff, pos=target_pos,
                        tol=self.tolerance, gains=self.natural_gains))

class QItselfActivity(Activity):
    def __init__(self, joint, q):
        super(QItselfActivity, self).__init__()
        assert_valid_joints(joint, joints())
        self.joint = joint
        self.q = q
        self.tolerance = .01
        self.moduloTwoPi = True

    def __str__(self):
        return ("(FollowReferenceActivity qItself {name})"
                "{{ type=qItself ref1={joint} target=[{target}] tol={tol} "
                "moduloTwoPi={modulo} PD={gains} }}"
                .format(name=self.name, joint=self.joint, target=self.q,
                        tol=self.tolerance,
                        modulo="1" if self.moduloTwoPi else "0",
                        gains=self.natural_gains))


class GazeAtActivity(Activity):
    def __init__(self, shape):
        super(GazeAtActivity, self).__init__()
        assert_valid_shapes(shape, shapes())
        self.shape = shape

    def __str__(self):
        pos = pos_str2arr(shapes(self.shape)["pos"])
        return ("(FollowReferenceActivity gazeAt {name})"
                "{{ type=gazeAt ref1=endeffHead ref2=base_footprint "
                "vec1=[0, 0, 1] vec2={pos} tol={tol} PD={gains}}}"
                .format(name=self.name, pos=pos, tol=self.tolerance,
                        gains=self.natural_gains))


class ReachActivity(Activity):
    def __init__(self, endeff, goal_shape, offset=None):
        super(ReachActivity, self).__init__()
        assert_valid_shapes(endeff, shapes())
        assert_valid_shapes(goal_shape, shapes())
        self.offset = offset
        if self.offset is None:
            self.offset = [0, 0, 0]
        self.endeff = endeff
        self.goal_shape = goal_shape

    def __str__(self):
        return ("(FollowReferenceActivity {endeff} {goal} {name})"
                "{{ type=pos ref1={endeff} ref2={goal} tol={tol} "
                "target={offset} PD={gains} }}"
                .format(name=self.name, endeff=self.endeff,
                        goal=self.goal_shape, tol=self.tolerance,
                        offset=self.offset, gains=self.natural_gains))


class AlignActivity(Activity):
    def __init__(self, endeff, vec_endeff, vec_target, name=None):
        super(AlignActivity, self).__init__()
        assert_valid_shapes(endeff, shapes())
        self.endeff = endeff
        self.vec_endeff = vec_endeff
        self.vec_target = vec_target
        if name is not None:
            self._name = name

    def __str__(self):
        return ("(FollowReferenceActivity {ref1} rot {name})"
                "{{ type=vec, ref1={ref1}, vec1={vec_endeff}, "
                "target={vec_target} }}"
                .format(name=self.name, ref1=self.endeff,
                        vec_endeff=self.vec_endeff,
                        vec_target=self.vec_target))


class HomingActivity(Activity):
    def __init__(self):
        super(HomingActivity, self).__init__()
        self.tolerance = .04

    def __str__(self):
        return ("(HomingActivity){{ tol={tol} PD={gains} }}"
                .format(tol=self.tolerance, gains=self.natural_gains))






###############################################################################
# Python activities
def homing():
    return HomingActivity()


def open_gripper(side=None):
    joint = side2gripper_joint(side)
    return QItselfActivity(joint, .08)


def close_gripper(side=None):
    joint = side2gripper_joint(side)
    return QItselfActivity(joint, .01)


def reach(what, with_=None, offset=None):
    """bla"""
    if with_ is None:
        with_ = side2endeff()
    if offset is None:
        offset = [0., 0., 0.]

    assert_valid_shapes(what, shapes())
    assert_valid_shapes(with_, shapes())

    return ReachActivity(with_, what, offset=offset)


def align_gripper_horizontal(side=None):
    return align_gripper([0, 0, 1], [0, 0, -1], side)


def align_gripper_vertical(side=None):
    return align_gripper([0, 1, 0], [0, 0, -1], side)


def align_gripper(vec_endeff, vec_target, side=None):
    endeff = side2endeff(side)

    assert_valid_shapes(endeff, shapes())

    return AlignActivity(endeff, vec_endeff, vec_target)


def align_gripper_with_plane(front_opening, rotation_around_wrist, side=None):
    endeff = side2endeff(side)

    assert_valid_shapes(endeff, shapes())

    return (AlignActivity(endeff, [1, 0, 0], front_opening, "front"),
            AlignActivity(endeff, [0, 1, 0], rotation_around_wrist, "rot"))


def gaze_at(shape):
    assert_valid_shapes(shape, shapes())

    return GazeAtActivity(shape)


def move_along_axis(endeff, start_pos, axis, distance):
    axis = np.asarray(axis)
    target_pos = start_pos + distance/np.linalg.norm(axis) * axis

    return PosActivity(endeff, pos=target_pos)


def turn_wrist(rel_degree, side=None):
    joint = side2wrist_joint(side)
    current_q = float(joints(joint)["q"])
    target = current_q + np.deg2rad(rel_degree)

    activity = QItselfActivity(joint, target)
    activity.moduloTwoPi = False
    return activity


def move_to_pos(endeff, pos):
    return PosActivity(endeff, pos)


###############################################################################
# High Level Behaviors
def grab_marker(shape, side=None):
    endeff = side2endeff(side)

    return [{"with": gaze_at(endeff),
             "plan": [(open_gripper(side),
                       reach(shape, offset=[0, .01, .1], with_=endeff),
                       align_gripper_with_plane([1, 0, 0], [0, -1, 0],
                                                side=side)
                       ),
                      reach(shape, offset=[0, .01, -.07], with_=endeff),
                      close_gripper(side)
                      ]
             }
            ]


def turn_marker(shape, degree, pre_grasp_offset=None, grasp_offset=None,
                plane=None, side=None):
    if pre_grasp_offset is None:
        pre_grasp_offset = [0, 0, 0]
    if grasp_offset is None:
        grasp_offset = [0, 0, 0]
    if plane is None:
        plane = ([1, 0, 0], [0, -1, 0])

    endeff = side2endeff(side)

    return [{"with": gaze_at(endeff),
             "plan": [(open_gripper(side),
                       reach(shape, with_=endeff, offset=pre_grasp_offset),
                       align_gripper_with_plane(*plane, side=side)),
                      reach(shape, endeff, offset=grasp_offset),
                      close_gripper(side),
                      turn_wrist(degree, side),
                      open_gripper(side)]
             }]


def move_shape(shape, distance, axis, pre_grasp_offset=None, grasp_offset=None,
               plane=None, side=None):
    if pre_grasp_offset is None:
        pre_grasp_offset = [0, 0, 0]
    if grasp_offset is None:
        grasp_offset = [0, 0, 0]
    if plane is None:
        plane = ([1, 0, 0], [0, -1, 0])

    endeff = side2endeff(side)

    return [align_gripper_with_plane(*plane, side=side),
            {"with": [align_gripper_with_plane(*plane, side=side),
                      gaze_at(endeff)],
             "plan": [(open_gripper(side),
                       reach(shape, offset=pre_grasp_offset, with_=endeff)),
                      reach(shape, offset=grasp_offset, with_=endeff),
                      close_gripper(side),
                      MoveAlongAxisActivity(endeff, axis, distance),
                      open_gripper()]
             }]


def move_shape_along_joint(shape, distance, joint, pre_grasp_offset=None,
                           grasp_offset=None, plane=None, side=None):
    axis = pos_str2arr(interface.getJointByName(joint)["axis"])
    print("Axis: {}".format(axis))
    return move_shape(shape, distance, axis, pre_grasp_offset, grasp_offset,
                      plane=plane, side=side)

print("Loaded actions.py...")

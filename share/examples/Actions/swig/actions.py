from __future__ import print_function

from contextlib import contextmanager
import numpy as np

from resources import interface, shapes, s, bodies, b, joints, j
from utils import (
    flatten,
    SIDE,
    strip_specs,
    conv_symbol,
    assert_in,
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

    interface.waitForAllCondition(symbols_conv)

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

    facts = flatten(facts)

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


def run(plan):
    """
    Runs a plan in :ref:`section-plan-format`


    :param plan: A plan in the plan format
    :return:
    """
    if not isinstance(plan, list):
        plan = [plan]
    for item in plan:
        if isinstance(item, list):
            run(item)
        elif isinstance(item, dict):
            _run_with(item)
        elif isinstance(item, tuple):
            _run(flatten(item))
        else:
            _run([item])

###############################################################################
# Python activitiy classes

class Activity(object):
    """
    An Activity is something which can be run on the robot and moves certain
    parts of it. This is the base class for it, which serves as a data
    container for common parameters.

    See the subclasses for details on what exactly they do.

    All Activities can be represented as strings, which can be fed to the
    relational machine.

    The subclasses of Activity must overwrite the __str__ method to return
    a string representation of an action.
    """
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

    def run(self):
        """Run this Activity"""
        run(str(self))


class PosActivity(Activity):
    """
    An Activity that moves an endeffector shape to a position in world
    coordinates.
    """
    def __init__(self, endeff, pos):
        """
        :param endeff: The endeffector shape to move
        :param pos: The target position in world coordinates
        :return:
        """
        super(PosActivity, self).__init__()
        assert_in(endeff, shapes())
        self.endeff = endeff
        self.pos = pos

    def __str__(self):
        return ("(FollowReferenceActivity {endeff} pos {name})"
                "{{ type=pos ref1={endeff} vec2={pos} tol={tol} PD={gains} }}"
                .format(name=self.name, endeff=self.endeff, pos=self.pos,
                        tol=self.tolerance, gains=self.natural_gains))


class MoveAlongAxisActivity(Activity):
    """
    Moves an endeffector shape a ceratin distance on a given axis. Note that
    the position is evaluated when the string is generated. Thus call run or
    __str__() only when you are ready to move.
    """
    def __init__(self, endeff, axis, distance):
        """
        :param endeff: The endeffector shape to move
        :param axis: The axis to move along in world coordinates
        :param distance: The distance to move the endeffector in meter
        :return:
        """
        super(MoveAlongAxisActivity, self).__init__()
        self.endeff = endeff
        self.axis = np.asarray(axis)
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
    """
    Moves a given joint to a specified q value.
    """
    def __init__(self, joint, q, moduloTwoPi=True):
        """
        :param joint: The name of the joint to move
        :param q: The desired position. For rotational joints in radian. For
                  translational joints in meter.
        :param moduloTwoPi: If set tu True all angles are set to values
                            between 0 and two pi radian or 360 degree
                            respectively. E.g. if you want to move a joint to
                            361 degree it moves to 1 degree. If set to False
                            it also makes multiple turns.
        :return:
        """
        super(QItselfActivity, self).__init__()
        assert_in(joint, joints())
        self.joint = joint
        self.q = q
        self.tolerance = .01
        self.moduloTwoPi = moduloTwoPi

    def __str__(self):
        return ("(FollowReferenceActivity qItself {name})"
                "{{ type=qItself ref1={joint} target=[{target}] tol={tol} "
                "moduloTwoPi={modulo} PD={gains} }}"
                .format(name=self.name, joint=self.joint, target=self.q,
                        tol=self.tolerance,
                        modulo="1" if self.moduloTwoPi else "0",
                        gains=self.natural_gains))


class TiltHead(QItselfActivity):
    """Tilt the head up (positive values) or down (negative values)."""

    def __init__(self, deg_relative=0):
        """
        :param deg_relative: The angle to turn the head up/down
        :return:
        """
        self.joint_name = "head_tilt_joint"
        super(TiltHead, self).__init__(self.joint_name, 0)
        self.radian_offset = np.deg2rad(deg_relative)

    def __str__(self):
        self.q = (float(interface.getJointByName(self.joint_name)["q"])
                  + self.radian_offset)
        return super(TiltHead, self).__str__()


class PanHead(QItselfActivity):
    """Pan/turn the head left (positive values) or right (negative values)."""

    def __init__(self, deg_relative=0):
        """
        :param deg_relative: The angle in degree to turn the head left
                             (positive values) or right (negative values).
        :return:
        """
        self.joint_name = "head_pan_joint"
        super(PanHead, self).__init__(self.joint_name, 0)
        self.radian_offset = np.deg2rad(deg_relative)

    def __str__(self):
        self.q = (float(interface.getJointByName(self.joint_name)["q"])
                  + self.radian_offset)
        return super(PanHead, self).__str__()


class GazeAtActivity(Activity):
    """
    Look at a shape.
    """
    def __init__(self, shape, pos_in_shape=None):
        """
        :param shape: The shape to look at
        :param pos_in_shape: Where in the shape to look. In coordinates of the
                             shape's own coordinate system. Default: [0, 0, 0]
        :return:
        """
        super(GazeAtActivity, self).__init__()
        assert_in(shape, shapes())
        self.shape = shape
        self.tolerance = .01
        self.position_in_shape = ([0, 0, 0]
                                  if pos_in_shape is None
                                  else pos_in_shape)

    def __str__(self):
        return ("(FollowReferenceActivity gazeAt {name})"
                "{{ type=gazeAt ref1=endeffHead "
                "ref2={target} "
                "vec1=[0, 0, 1] vec2={pos} tol={tol} PD={gains}}}"
                .format(name=self.name, target=self.shape, tol=self.tolerance,
                        pos=self.position_in_shape, gains=self.natural_gains))


class ReachActivity(Activity):
    """
    Reach a shape with a given endeffector shape.
    """
    def __init__(self, endeff, goal_shape, offset=None):
        """
        :param endeff: The endeffector shape to reach the shape with.
        :param goal_shape: The shape to reach
        :param offset: A offset to the shapes position in the goal shape's
                       coordinate system. Default: [0, 0, 0]
        :return:
        """
        super(ReachActivity, self).__init__()
        assert_in(endeff, shapes())
        assert_in(goal_shape, shapes())
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
    """
    Align a vector attached to an endeffector with a given target vector
    """
    def __init__(self, endeff, vec_endeff, vec_target, name=None):
        """
        :param endeff: The endeffector shape to align
        :param vec_endeff: The vector in the endeffector's coordinate system
                           attached to the endeffect, which is to be aligned
        :param vec_target: The target vector in world coordinates
        :param name: A name for this activity
        """

        super(AlignActivity, self).__init__()
        assert_in(endeff, shapes())
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
    """
    Move the robot in the base position. The base does _not_ move to [0, 0, 0]
    in world coordinates.
    """
    def __init__(self):
        super(HomingActivity, self).__init__()
        self.tolerance = .08

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

    assert_in(what, shapes())
    assert_in(with_, shapes())

    return ReachActivity(with_, what, offset=offset)


def align_gripper_horizontal(side=None):
    return align_gripper([0, 0, 1], [0, 0, -1], side)


def align_gripper_vertical(side=None):
    return align_gripper([0, 1, 0], [0, 0, -1], side)


def align_gripper(vec_endeff, vec_target, side=None):
    endeff = side2endeff(side)

    assert_in(endeff, shapes())

    return AlignActivity(endeff, vec_endeff, vec_target)


def align_gripper_with_plane(front_opening, rotation_around_wrist, side=None):
    endeff = side2endeff(side)

    assert_in(endeff, shapes())

    return (AlignActivity(endeff, [1, 0, 0], front_opening, "front"),
            AlignActivity(endeff, [0, 1, 0], rotation_around_wrist, "rot"))


def gaze_at(shape):
    assert_in(shape, shapes())

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

"""
The ``actions module`` contains
(a) low level actHomogenions the robot can perform,
(b) methods to run and sequence these actions,
(c) high level behaviors that use low level actions and and the sequencing
    stuff.

All implementations of activities must inherit from the base class
``Activity``.
"""
from __future__ import print_function

from contextlib import contextmanager
from collections import namedtuple
import numpy as np
import time
import sys
import signal


from resources import interface, shapes, bodies, joints, facts
from utils import (
    flatten,
    SIDE,
    Bg_facts,
    strip_specs,
    conv_symbol,
    assert_in,
    side2endeff,
    side2gripper_joint,
    pos_str2arr,
    side2wrist_joint,
    dehomogenize,
    quaternion_matrix,
)

symbole = []
symbole_conv = []
s = []
b = []
j = []

def signal_handler(signal, frame):
    print('ABORT!')
    global symbole
    global symbole_conv

    for symb in reversed(symbole):
        print("#############################################" +symb)
        interface.stopFact(symb)
        x = symbole.pop()


    active_facts = interface.getFacts()
    for symb_conv in reversed(symbole_conv):
        x = symbole_conv.pop()
        if symb_conv + "," in active_facts:
            interface.stopFact(symb_conv)
            

signal.signal(signal.SIGABRT, signal_handler)





###############################################################################
# Convenient access and autocompletion to shapes, joints, and bodies
# Just type `s.<tab>` to get a list of all shapes
def  update_a():
    _tmp = list(shapes())
    Shapes = namedtuple("Shapes", " ".join(_tmp))

    s = Shapes(*_tmp)

    _tmp = list(bodies())
    Bodies = namedtuple("Bodies", " ".join(_tmp))
    b = Bodies(*_tmp)

    _tmp = list(joints())
    Joints = namedtuple("Joints", " ".join(_tmp))
    j = Joints(*_tmp)
    return [s,b,j]
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
    """Excecutes a plan in the plan format.

    :param with_construct: The with-construct
    """
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


def run_in_bg(facts):
    """Run given facts in background.

    :param facts: List of facts to excecute.

    """
    global symbole
    global symbole_conv
    if not isinstance(facts, list):
        facts = [facts]
    facts = flatten(facts)

    symbols = [strip_specs(str(fact)) for fact in facts]
    symbols_conv = [conv_symbol(symbol) for symbol in symbols]

    symbole += symbols
    symbole_conv += symbols_conv

    for fact in facts:
        interface.setFact(str(fact))
        
        

def remove_facts():

    """Remove all active facts an conv-symbols.

    """
    facts = interface.getFacts()
    symbols = [strip_specs(str(fact)) for fact in facts]
    for symb in symbols:
        interface.stopFact(symb)


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
    id = 0
    def __init__(self):
        self.time = 3.
        self.damping = .7
        self.max_vel = 10
        self.max_acc = 10
        self.tolerance = .01
        self._name = ""
        self.id = Activity.id
        Activity.id += 1
        interface.createNewSymbol(str(self.id))

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

class AllActivity(Activity):
    """
    An activity that converts given parameters to a fact (string) and excecutes it.
    """
    
    def __init__(self, ref1, ref2=None, vec1=None, vec2=None, target=None, type="pos", moduloTwoPi=False):
        """
        :param ref1: The endeffector shape to move.
            A joint in case of "qItself
        :param ref2: The shape to move the endeffector relative to.
        :param vec1: The vector of ref1 to align.
        :param vec2: The vector of ref2 as the align-reference-vector.
        :param target: The target position of ref1 in coordinates of ref2.
        :param type: The different types of movement:

            * **pos:** Move ref1 to position (target) in coordinates of ref2. If ref2 is not set, in world coordinates.
            
            * **vec:** Not implemented yet.
            
            * **vecDiff:** Align vec1 of ref1 with vec2 of ref2. vec1 and vec2 must be normalized.
            
            * **gazeAt:** Point vec1 of ref1 to pos (target) of ref2. If target not set, vec1 points to ref2.
            
            * **qItself:** Align joint(ref1) to a given angle (target).
            
            * **wheels:** Moves and rotates Base to a position (target) in world coordinates. target[0]: X-coordinate. target[1]: Y-coordinate. target[2]: Rotation.
            
            * **homing:** Homing Activity.
        
        :param moduloTwoPi: If set tu True all angles are set to values
            between 0 and two pi radian or 360 degree
            respectively. E.g. if you want to move a joint to
            361 degree it moves to 1 degree. If set to False
            it also makes multiple turns.
        :return:
        """
        super(AllActivity, self).__init__()
        self.ref1 = ref1
        self.ref2 = ref2
        self.vec1 = vec1
        self.vec2 = vec2
        self.type = type
        self.target = target
        self.moduloTwoPi = moduloTwoPi
        if self.ref1 + "," not in interface.getSymbols():
            interface.createNewSymbol(self.ref1)

    def __str__(self):
        return ("(FollowReferenceActivity {ref}{type}{name}{id})"
                "{{ type={type} {ref1} {ref2} {target} {vec1} {vec2} tol={tol} PD={gains} {moduloTwoPi} }}"
                .format(name=self.name + " " if self.name else "",
                        type=self.type + " ", ref=self.ref1 + " ", id=self.id,
                        ref1="ref1=" + self.ref1 if self.ref1 else "",
                        ref2="ref2=" + self.ref2 if self.ref2 else "",
                        target="target=" + str(self.target) if self.target else "",
                        vec1="vec1=" + str(self.vec1) if self.vec1 else "",
                        vec2="vec2="+ str(self.vec2) if self.vec2 else "",
                        moduloTwoPi="moduloTwoPi=1" if self.moduloTwoPi else "",
                        tol=self.tolerance, gains=self.natural_gains))


class PosActivity(AllActivity):
    """
    An Activity that moves an endeffector shape relative to its coordinate system.
    If relative is true. Movement is relative to the base coordinates.
    """
    def __init__(self, ref1, q, relative=False):
        """
        :param ref1: The endeffector shape to move
        :param q: The target position
        :param relative: If set, q is in Base coordinates.
            If not set, q is in ref1 coordinates
        :return:
        """
        assert_in(ref1, shapes())
        q.append(1)
        endeff = ref1 if relative else "endeffBase"
        q = np.dot(quaternion_matrix(pos_str2arr(shapes(endeff)["Q"]).tolist()), q) 
        target = np.add(pos_str2arr(shapes(ref1)["pos"]).tolist(), dehomogenize(q)).tolist()
        
        super(PosActivity, self).__init__(ref1=ref1, target=target)
        

    def __str__(self):
        return super(PosActivity, self).__str__()


class VecActivity(AllActivity):
    """
    An Activity that aligns an axis of a shape with an axis in world coordinates.
    """
    def __init__(self, ref1, vec1, target):
        """
        :param ref1: The shape to align.
        :param vec1: The normalize axis of ref1 to align
        :param target: the normalize axis to align to in world coordinates
        :return:
        """
        assert_in(ref1, shapes())
        super (VecActivity, self).__init__(ref1=ref1, vec1=vec1, target=target, type="vec")
        
    def __str__(self):
        return super(VecActivity, self).__str__()

class VecDiffActivity(AllActivity):
    """
    An Activity that aligns an axis of a shape to an axis of another shape.
    """
    def __init__(self, ref1, ref2, vec1=[1,0,0], vec2=[-1,0,0]):
        """
        :param ref1: The shape to align
        :param ref2: The shape to align to
        :param vec1: The normalized axis of ref1 to align
        :param vec2: the normalized axis of ref2 to align to
        :param pos: The target position in world coordinates
        :return:
        """
        assert_in(ref1, shapes())
        assert_in(ref2, shapes())
        super(VecDiffActivity, self).__init__(ref1=ref1, 
            ref2=ref2, vec1=vec1, vec2=vec2, type="vecDiff")
        

    def __str__(self):
        return super(VecDiffActivity, self).__str__()



class QItselfActivity(AllActivity):
    """
    An Activity that moves a given joint to a specified q value.
    """
    def __init__(self, ref1, target, moduloTwoPi=True):
        """
        :param ref1: The name of the joint to move
        :param target: The desired position. For rotational joints in radian. For
            translational joints in meter.
        :param moduloTwoPi: If set tu True all angles are set to values
                            between 0 and two pi radian or 360 degree
                            respectively. E.g. if you want to move a joint to
                            361 degree it moves to 1 degree. If set to False
                            it also makes multiple turns.
        :return:
        """
        assert_in(ref1, joints())
        if not isinstance(target, list):
            target = [target]
        super(QItselfActivity, self).__init__(ref1=ref1, target=target, type="qItself", moduloTwoPi=moduloTwoPi)


    def __str__(self):
        return super(QItselfActivity, self).__str__()


class WheelsActivity(AllActivity):
    """

    An Activity that moves the robot to a specified position relative to the base.
    """
    def __init__(self, q, absolute=False):
        """
        :param q: The position to move relative to current position.
            q[0]: X-coordinate. q[1]: Y-coordinate. q[2]: Rotation.
        :param absolute: Absolute position flag. If set, q is in world coordinates.
        :return:
        """

        if absolute:
            target = q
        else:
            q_base = [float(x) for x in joints("worldTranslationRotation")["q"].split()]
            target = [0, 0, 0]
            target[0] = np.sin(q_base[2])*-q[1]+np.cos(q_base[2])*q[0]
            target[1] = np.sin(q_base[2])*q[0]+np.cos(q_base[2])*q[1]
            target[2] = np.deg2rad(q[2])
            target = [x + y for x, y in zip(target, q_base)]
        super(WheelsActivity, self).__init__(target=target, type="wheels",moduloTwoPi=True)

    def __str__(self):   
        return super(WheelsActivity, self).__str__()


class GazeAtActivity(AllActivity):
    """
    An activity that points an axis of of shape to another shape.
    """
    def __init__(self, ref1, ref2, vec1=[1,0,0], vec2=[0,0,0]):
        """
        :param ref1: The shape to align
        :param ref2: The shape to point at
        :param vec1: The normalized axis of vec1 to align. Default: [1,0,0]
        :param vec2: Where in the shape to look. In coordinates of the
                             shape's own coordinate system. Default: [0, 0, 0]
        :return:
        """
        
        assert_in(ref1, shapes())
        assert_in(ref2, shapes())
        
        super(GazeAtActivity, self).__init__(ref1=ref1,ref2=ref2,vec1=vec1, vec2=vec2, type="gazeAt")

    def __str__(self):
        return super(GazeAtActivity, self).__str__()

class MoveBaseToShape(WheelsActivity):
    """Move base in front of shape with given offset"""

    def __init__(self, shape, offset=1):
        """
        :param shape: The destination of the movement
        :param offset: The distance to reach between base and shape in meter.
        :return:
        """
        assert_in(shape, shapes())
        pos = pos_str2arr(interface.getShapeByName(shape)["pos"]).tolist()
        X = np.array(pos_str2arr(interface.getShapeByName(shape)["Z"]))
        X = np.cross(np.cross(self.X,[0,0,1]),[0,0,1])
        X = np.divide(X,np.linalg.norm(X))
        pos[2] = np.arccos(np.dot([1,0,0],X))
        X = np.multiply(X, offset)
        pos[0] -= X[0]
        pos[1] -= X[1] 

        super(MoveBaseToShape, self).__init__(pos, 1)
        

    def __str__(self):
        return super(MoveBaseToShape, self).__str__()

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


class TiltHead(QItselfActivity):
    """Tilt the head up (positive values) or down (negative values)."""

    def __init__(self, up_deg_relative=0):
        """
        :param up_deg_relative: The angle to turn the head up/down
        :return:
        """
        self.joint_name = "head_tilt_joint"
        super(TiltHead, self).__init__(self.joint_name, 0)
        self.radian_offset = np.deg2rad(up_deg_relative)

    def __str__(self):
        self.q = (float(interface.getJointByName(self.joint_name)["q"])
                  + self.radian_offset)
        return super(TiltHead, self).__str__()


class PanHead(QItselfActivity):
    """Pan/turn the head left (positive values) or right (negative values)."""

    def __init__(self, left_deg_relative=0):
        """
        :param left_deg_relative: The angle in degree to turn the head left
                             (positive values) or right (negative values).
        :return:
        """
        self.joint_name = "head_pan_joint"
        super(PanHead, self).__init__(self.joint_name, 0)
        self.radian_offset = np.deg2rad(left_deg_relative)

    def __str__(self):
        self.q = (float(interface.getJointByName(self.joint_name)["q"])
                  + self.radian_offset)
        return super(PanHead, self).__str__()


class LookAt(GazeAtActivity):
    """
    Look at a shape.
    """
    def __init__(self, ref2, vec2=[0,0,0]):
        """
        :param shape: The shape to look at
        :param pos_in_shape: Where in the shape to look. In coordinates of the
                             shape's own coordinate system. Default: [0, 0, 0]
        :return:
        """
        
        #assert_in(ref2, shapes())
        self.ref2 = ref2
        self.tolerance = .01
        self.vec2 = vec2
        super(LookAt, self).__init__(ref1="endeffHead", ref2=self.ref2, vec1=[0,0,1], vec2=self.vec2)

    def __str__(self):
        return super(LookAt, self).__str__()


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


def forward(meters):
    return WheelsActivity([meters,0,0])

def side(meters):
    return WheelsActivity([0,meters,0])

def turn(degrees):
    return WheelsActivity([0,0, degees])


def align_gripper_with_marker(marker,side=None):
    return  (VecDiffActivity(side2endeff(side),marker),  VecDiffActivity(side2endeff(side),marker,[0,1,0],[0,1,0]))

def ding(marker):
    return [{"with":[align_gripper_with_marker(marker), LookAt(marker)],
            "plan":[reach(marker),open_gripper()]}]

def door(shape):
    return [{"with":(),
        "plan":[homing(),MoveBaseToShape(shape),align_gripper_vertical(SIDE.LEFT),{
            "with": [LookAt(shape), align_gripper_vertical(SIDE.LEFT)],
            "plan": [AllActivity("endeffL",shape,target=[-0.15,-0.35,0.20]),
                    open_gripper(SIDE.LEFT),
                    AllActivity("endeffL",shape,target=[-0.15,-0.35,0.10]),
                    close_gripper(SIDE.LEFT),
                    ]},
            AllActivity("endeffL",shape,target=[-0.25,-0.25,0.20]),
            AllActivity("endeffL",shape,target=[-0.25,-0.25,0.20]),]

             }]

def homing():
    return HomingActivity()


def open_gripper(side=None):
    joint = side2gripper_joint(side)
    return QItselfActivity(joint, .1)


def close_gripper(side=None):
    joint = side2gripper_joint(side)
    return QItselfActivity(joint, .01)


def reach(what, with_=None, offset=None):
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

def align_gripper_with_shape(shape, side=None):
    endeff = side2endeff(side)
    return GazeAtActivity(ref1=endeff, ref2=shape, vec1=[1,0,0])


def gaze_at(shape):
    assert_in(shape, shapes())

    return LookAt(shape)


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

    return [{"with": gaze_at(shape),
             "plan": [(open_gripper(side),
                       reach(shape, offset=[-0.05, 0.05, 0.1], with_=endeff),
                       align_gripper_with_plane([1, 0, 0], [0, -1, 0],
                                                side=side)
                       ),
                      reach(shape, offset=[-0.05, 0.05, -0.1], with_=endeff),
                      close_gripper(side)
                      ]
             }
            ]
def in_front(shape, side=None):
    endeff = side2endeff(side)

    return [{"with": [gaze_at(shape), 
                GazeAtActivity(endeff, shape, [1, 0, 0]), 
                #VecDiffActivity(endeff, shape, [0, 1, 0], [0,-1,0])
                ],
             "plan": reach(shape, offset=[0, 0, .1], with_=endeff)
            }]

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


#update()

print("Loaded actions.py...")

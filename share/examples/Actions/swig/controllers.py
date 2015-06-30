from __future__ import division

from enum import Enum

from actions import *
from utils import SIDE

class LockboxJoint(Enum):
    door = 0
    bar = 1
    wheel = 2
    bolt = 3
    screw = 4
    pin = 5


def move(joint, pos):
    if joint == LockboxJoint.pin:
        return pin_controller(pos)
    elif joint == LockboxJoint.screw:
        return screw_controller(pos)
    elif joint == LockboxJoint.bolt:
        return bolt_controller(pos)
    elif joint == LockboxJoint.wheel:
        return wheel_controller(pos)
    elif joint == LockboxJoint.bar:
        return bar_controller(pos)
    else:
        raise ValueError("{} not a controllable joint.".format(joint))


def pin_controller(pos):
    current_pos = int(interface.getJointByName(j.lockbox_pin)["q"])
    distance = pos - current_pos
    pre_grasp_offset = [-.1, 0, 0]
    run(move_shape_along_joint(s.big_pin, distance, j.lockbox_pin,
                               pre_grasp_offset=pre_grasp_offset))
    run(homing())


def bolt_controller(pos):
    current_pos = int(interface.getJointByName(j.lockbox_bolt)["q"])
    distance = pos - current_pos
    pre_grasp_offset = [-.1, 0, 0]
    run(move_shape_along_joint(s.big_bolt, distance, j.lockbox_bolt,
                               pre_grasp_offset=pre_grasp_offset))
    run(homing())


def bar_controller(pos):
    current_pos = int(interface.getJointByName(j.lockbox_bar)["q"])
    distance = pos - current_pos
    pre_grasp_offset = [0, -.1, 0]
    plane = ([1, 0, 0], [0, 0, 1])
    run(move_shape_along_joint(s.big_bar, distance, j.lockbox_bar,
                               pre_grasp_offset=pre_grasp_offset,
                               plane=plane))
    run(homing())


def wheel_controller(pos):
    print("Not yet...")
    return
    current_pos = int(interface.getJointByName(j.lockbox_wheel)["q"])
    distance = pos - np.rad2deg(current_pos)
    print("Distance: {}".format(distance))
    run(turn_marker(s.wheel_handle, distance))
    run(homing())


def screw_controller(pos):
    print("Not yet...")
    return
    side = SIDE.RIGHT
    endeff = side2endeff(side)
    joint = side2wrist_joint(side)
    shape = s.head

    # a M10 screw moves 1.5 mm per turn, see:
    # http://en.wikipedia.org/wiki/ISO_metric_screw_thread
    turn_distance = .0015

    current_pos = int(interface.getJointByName(j.lockbox_screw)["q"])
    distance = pos - current_pos

    current_q = float(joints(joint)["q"])

    turns = distance / turn_distance
    degree = -360 * turns

    axis = pos_str2arr(interface.getJointByName(j.lockbox_screw)["axis"])

    pre_grasp_offset = [0, -.05, 0]
    plan = turn_marker(shape, degree=degree, pre_grasp_offset=pre_grasp_offset,
                       plane=(axis, [0, 1, 0]), side=SIDE.RIGHT)
    simultan = plan[0]["plan"][0]
    plan[0]["plan"][0] = (simultan[0], simultan[1],
                          align_gripper([1, 0, 0], [0, 1, 0], side=side))
    plan[0]["plan"][-2].time = turns

    plan.append(reach(shape, offset=pre_grasp_offset, with_=endeff))
    # plan.append(homing())

    run(plan)
    interface.resetHighValue(joint)

    # with running(align_gripper([1, 0, 0], [0, 1, 0], side=side) +
    #              gaze_at(endeff)):
    #     run(open_gripper(side=side)
    #         + reach(shape, offset=[0.0, -0.05, 0.0], with_=endeff))
    #     run(reach(shape, offset=[0.0, .0, 0.], with_=endeff))
    #     run(close_gripper(side=side))
    #     run(move_along_axis(endeff, axis, -distance) +
    #         [fact])
    #     run(open_gripper(side=side))
    #     run(reach(shape, offset=[0.0, -0.05, 0.0], with_=endeff))
    #
    # run(homing())

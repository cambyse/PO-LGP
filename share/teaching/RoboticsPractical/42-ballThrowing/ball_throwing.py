#! /usr/bin/env python2
# Import python modules
import rospy as rp
import argparse
import baxter_interface as bax
import time
import thread
import alvar_marker_test
#import os
from baxter_interface import Gripper
import numpy as np

# Initialise such that it registers somehow or something, I don't know...
rp.init_node('Hello_Baxter')
limb = bax.Limb('left')
left_gripper = Gripper('left')

# Get joint startpos_1
startpos = dict()
startpos['left_w0'] = -0.7834806874124751
startpos['left_w1'] = -0.3857961681531816
startpos['left_w2'] = -2.6349954983901696
startpos['left_e0'] = -2.5713352956929247
startpos['left_e1'] =  0.25732527716777814
startpos['left_s0'] = -0.45674277959288195
startpos['left_s1'] =  1.04272344056511

#def disable_gravity():
#    os.execv('/opt/ros/indigo/bin/rostopic', ['rostopic', 'pub', '-r', '10',
#    '/robot/limb/left/suppress_gravity_compensation', 'std_msgs/Empty'])

# u: force


def dict2npa(u):
    na = np.array([u['left_w1'],
                   u['left_e1'],
                   u['left_s1']])
    return na

def npa2dict(u)
    return {'left_w0': 0,
            'left_w1': u[0],
            'left_w2': 0,
            'left_e0': 0,
            'left_e1': u[1],
            'left_s0': 0,
            'left_s1': u[2]}

def features(t):
    u = mk_np_array(limb.joint_efforts())
    v = mk_np_array(limb.joint_velocities())
    p = mk_np_array(limb.joint_angles())

    #return np.concatenate((u, v, p))
    return np.ones(6) * t

def control(features, W):
    ub = lb = 0.8

    v = np.dot(W, features) + np.random.randn(3)
    v_dict = npa2dict(v)

    limits = {'left_w0': range(-3.059 * lb, 3.059 * ub),
            'left_w1': range(-1.5708 * lb, 2.094 * ub),
            'left_w2': range( -3.059 * lb, 3.059 * ub),
            'left_e0': range(-3.05418 * lb, 3.05418 * ub),
            'left_e1': range(-0.05 * lb, 2.618 * ub),
            'left_s0': range(-1.70168 * lb, 1.70168 * ub),
            'left_s1': range(-2.147 * lb, 1.047 * ub)}

    for key, value in limits.items:
        if not limb.joint_angle[key] in value:
            v_dict[key] = 0

    return v_dict


def start(E, T): 
    for e in range(E):
        for t in range(T):
            v = control(features(t), W)
            limb.set_joint_velocities(v)
            time.sleep(0.01)

def reset():
    limb.move_to_joint_positions(startpos)

def init_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--simulate', help="Don't use any ros connections, just simulate.", type=bool, default=0)
    return parser.parse_args()


if __name__ == "__main__":
    args = init_parser()

    # Start alvar listener thread thingy
    if (args.simulate == 0):
        thread.start_new_thread(alvar_marker_test.listener, ())
    print('All done')

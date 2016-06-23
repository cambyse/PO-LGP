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


#def disable_gravity():
#    os.execv('/opt/ros/indigo/bin/rostopic', ['rostopic', 'pub', '-r', '10',
#    '/robot/limb/left/suppress_gravity_compensation', 'std_msgs/Empty'])

# u: force


def dict2npa(u):
    na = np.array([u['left_w1'],
                   u['left_e1'],
                   u['left_s1']])
    return na

def npa2dict(u):
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
    v = np.dot(W, features) + np.random.randn(3)
    v_dict = npa2dict(v)

    for key, value in limits.items:
        if limb.joint_angle[key] < value[0] or limb.joint_angle[key] > value[1]:
            v_dict[key] = 0

    return v_dict

def expected_policy_reward(samples, T, W):
    episod_rewards = []
    for e in range(samples):
        for t in range(T):
            v = control(features(t), W)
            limb.set_joint_velocities(v)
            time.sleep(0.01)

        episod_reward.append(-alvar_marker_test.get_sq_dist())

    episod_rewards = np.array(episod_reward)
    return np.sum(episod_rewards)/episod_rewards.size

def start(E, T, W):
    er_prev = 0

    while True:
        noise = np.random.randn(W.shape[0], W.shape[1])
        er = expected_policy_reward(1, T, W + noise)

        if er > er_prev:
            er_prev = er
            W += noise


def reset():
    limb.move_to_joint_positions(startpos)

def init_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--simulate', help="Don't use any ros\
            connections, just simulate.", action='store_true', dest='simulate')
    parser.set_defaults(simulate=False)
    return parser.parse_args()


if __name__ == "__main__":
    # Get joint startpos_1
    startpos = dict()
    startpos['left_w0'] = -0.7834806874124751
    startpos['left_w1'] = -0.3857961681531816
    startpos['left_w2'] = -2.6349954983901696
    startpos['left_e0'] = -2.5713352956929247
    startpos['left_e1'] =  0.25732527716777814
    startpos['left_s0'] = -0.45674277959288195
    startpos['left_s1'] =  1.04272344056511
    ub = lb = 0.8

    
    limits = {'left_w0': [-3.059 * lb, 3.059 * ub],
            'left_w1': [-1.5708 * lb, 2.094 * ub],
            'left_w2': [ -3.059 * lb, 3.059 * ub],
            'left_e0': [-3.05418 * lb, 3.05418 * ub],
            'left_e1': [-0.05 * lb, 2.618 * ub],
            'left_s0': [-1.70168 * lb, 1.70168 * ub],
            'left_s1': [-2.147 * lb, 1.047 * ub]}

    args = init_parser()

    # Start alvar listener thread thingy
    print("Simulate value", args.simulate)
    if (args.simulate):
        print("Simulation.")
    else:
        thread.start_new_thread(alvar_marker_test.listener, ())
        # Initialise such that it registers somehow or something, I don't know...
        rp.init_node('Hello_Baxter')
        limb = bax.Limb('left')
        left_gripper = Gripper('left')

    print('All done')




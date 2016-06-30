#! /usr/bin/env python2
# Import python modules
import sys
import rospy as rp
import argparse
import baxter_interface as bax
import time
from baxter_interface import Gripper
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose

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
    '''
    u = dict2npa(limb.joint_efforts())
    v = dict2npa(limb.joint_velocities())
    p = dict2npa(limb.joint_angles())

    #return np.concatenate((u, v, p))
    '''
    t2 = t * t
    phi = np.array([t, t2, t, t2, t, t2])
    return phi

def control(features, W):
    v = np.dot(W, features) + np.random.randn(3)

    # Now, all joint forces are positive
    v_dict = npa2dict(np.abs(v))
    
    # There are some joints, where the velocity should be negative.
    # Those are turned into their negative counterpart.
    negative_joints = {'left_s1', 'left_e1', 'left_w1'}
    for joint in negative_joints:
        v_dict[joint] *= -1

    # Check for joint limits
    # If they are reached, set velocity of resp. angle 0.
    for key, value in limits.items():
        if limb.joint_angle(key) < value[0]:
            v_dict[key] = max(0, v_dict[key])
        elif limb.joint_angle(key) > value[1]:
            v_dict[key] = min(0, v_dict[key])

    return v_dict

''' Throws the Hacky Sack one time with T seconds,
    using W as weight matrix and a fixed time_step '''
def expected_policy_reward(T, W, time_step=5):
    reward = -1000
    limit = 15
    
    # Does all time step except for ${limit}
    for t in range(T/time_step - limit):
        v = control(features(t), W)
        send_signal(limb.set_joint_velocities, v, time_step)

    # Open the gripper to release the hacky sack
    left_gripper.command_position(100)

    # Should do ${limit} more time steps.
    # The basic idea is to keep moving while baxter
    # opens his grippers.
    for t in range(T/time_step - limit, T/time_step):
        v = control(features(t), W)
        send_signal(limb.set_joint_velocities, v, time_step)

    # Decelerate so that the joints get to a resting position
    # (hopefully quite fast)
    send_signal(limb.set_joint_velocities, zero_velocity, 1)

    # Don't measure distance if camera can't see the hacky sack
    c = raw_input('Measure distance? (m)')
    if c == 'm':
        # Measure until the camera catched both alvar markers
        # This is checked by waiting for an 'o'
        while True:
            sd = get_sq_dist()
            if sd:
                reward = -sd
                print('R = ' + str(reward))

            c = raw_input('Ok? (o)')
            if c == 'o':
                break

    return reward

''' Start the learning process '''
def start(T, W):
    # Initially VERY low optimum so that every real reward is better
    er_opt = -10000
    std_dev_init = 0.1
    std_dev = std_dev_init

    # Learn until we interrupt
    while True:
        # Sample new (gaussian) noise and throw the ball to get the reward.
        noise = np.random.randn(W.shape[0], W.shape[1])
        er = expected_policy_reward(T, W + std_dev * noise)

        # If the reward is better than our current optimum, remember this W.
        if er > er_opt:
            er_opt = er
            W += std_dev * noise
            std_dev = std_dev_init
        else:
            std_dev = std_dev * 2

        # Move arm back to start position
        send_signal(limb.set_joint_positions, startpos, 2000)

        # Wait for the hacky sack being between the gripper
        while True:
            c = raw_input('Hacky Sack (y)')
            if c == 'y':
                break

        # Grip if the hacky sack is in the right position.
        left_gripper.command_position(5)
        print(W)

        # Stop if user input asks to
        c = raw_input('Quit? (q)')
        if c == 'q':
            break
        else:
            while True:
                c = raw_input('Start? (s)')
                if c == 's':
                    break

'''Argument Parser'''
def init_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--simulate', help="Don't use any ros\
            connections, just simulate.", action='store_true', dest='simulate')
    parser.set_defaults(simulate=False)
    return parser.parse_args()

markers = dict()
keys = [10, 11]

''' Callback for the listener '''
def callback(data):
    # Just store the position of the marker.
    markers[data.id] = data.pose.position
    #rospy.loginfo(rospy.get_caller_id() + 'msg: %s', data.points)

''' Returns squared distance of the two markers defined in keys.
    Returns -1 in case of insufficient data.'''
def get_sq_dist():

    if keys[0] not in markers.keys() or keys[1] not in markers.keys() :
        print('Insufficient data.')
        print('Needed:', keys)
        print('Available:', markers.keys())
        return None
    else:
        marker1 = markers[keys[0]]
        marker2 = markers[keys[1]]
        return (marker2.x - marker1.x)**2 + (marker2.y - marker1.y)**2

''' Sends a signal to the robot for ${duration} seconds with
    a frequency of ${hz}. Calls ${func}. '''
def send_signal(func, joints, duration, hz=100):
    for t in range(duration*hz):
        func(joints)
        time.sleep(1/hz)


if __name__ == "__main__":
    # Get joint startpos_1
    startpos = dict()
    startpos['left_w0'] = -0.0625097171063306
    startpos['left_w1'] = -0.5000777368506448
    startpos['left_w2'] = -0.13460681413694506
    startpos['left_e0'] =  0.23048061337978343
    startpos['left_e1'] =  1.0745535419137324
    startpos['left_s0'] = -0.5518495884417776
    startpos['left_s1'] =  0.7374612637759127

    ub = lb = 0.7

    # First entry: lower limit, Second entry: upper limit.
    # All limits are bounded by ${lb} and ${ub}%.
    limits = {
            'left_w0': [-3.059   * lb, 3.059   * ub],
            'left_w1': [-1.5708  * lb, 2.094   * ub],
            'left_w2': [-3.059   * lb, 3.059   * ub],
            'left_e0': [-3.05418 * lb, 3.05418 * ub],
            'left_e1': [-0.05    * lb, 2.618   * ub],
            'left_s0': [-1.70168 * lb, 1.70168 * ub],
            'left_s1': [-2.147   * lb, 1.047   * ub]
            }

    # Vector with 0 velocity to stop the motion.
    zero_velocity = {
            'left_w0': 0.0,
            'left_w1': 0.0, 
            'left_w2': 0.0,
            'left_e0': 0.0,
            'left_e1': 0.0,
            'left_s0': 0.0,
            'left_s1': 0.0
            }

    args = init_parser()

    # Start alvar listener thread thingy
    print("Simulate value", args.simulate)
    if (args.simulate):
        print("Simulation.")
    else:
        # Initialise such that it registers somehow or something, I don't know...
        rp.init_node('Ball')
        rp.Subscriber('visualization_marker', Marker, callback)
        limb = bax.Limb('left')
        left_gripper = Gripper('left')
    
        send_signal(limb.set_joint_positions, startpos, 1500)
        while True:
            left_gripper.command_position(100)
            while True:
                c = raw_input('Hacky Sack (y)')
                if c == 'y':
                    break
            left_gripper.command_position(5)


            c = raw_input('Start? (s)')
            if c == 's':
                break
        W0 = (np.random.rand(3, 6) * 2 - 1)
        start(250, W0)

    print('All done')

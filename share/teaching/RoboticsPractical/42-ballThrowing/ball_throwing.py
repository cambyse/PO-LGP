#! /usr/bin/env python2
# Import python modules
import rospy as rp
import baxter_interface as bax
import time as t
#import thread
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
def apply_force(u):
    limb.set_joint_velocities(u)


def mk_np_array(u):
    na = np.array([u['left_w0'],
                    u['left_w1'],
                    u['left_w2'],
                    u['left_e0'],
                    u['left_e1'],
                    u['left_s0'],
                    u['left_s1']])
    return na


def features():
    u = mk_np_array(limb.joint_efforts())
    v = mk_np_array(limb.joint_velocities())
    p = mk_np_array(limb.joint_angles())

    return np.concatenate((u, v, p))


def control(features, w):
    u = dict()


    u['left_s0'] =  0.0
    u['left_s1'] = -1.0
    u['left_e0'] =  0.0
    u['left_e1'] =  1.0
    u['left_w0'] =  0.0
    u['left_w1'] = -1.0
    u['left_w2'] =  0.0

    #thread.start_new_thread(disable_gravity, ())

    for i in range(75):
     #   pub.publish("")
        apply_force(u)
        t.sleep(0.01)

    # Reset position
    u['left_s0'] =  0.0
    u['left_s1'] =  0.0
    u['left_e0'] =  0.0
    u['left_e1'] =  0.0
    u['left_w0'] =  0.0
    u['left_w1'] =  0.0
    u['left_w2'] =  0.0
    apply_force(u)

def reset():
    limb.move_to_joint_positions(startpos)

if __name__ == "__main__":
    print(type(limb.joint_angles()))
    print('All done')

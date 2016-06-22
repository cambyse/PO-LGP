#! /usr/bin/env python2
# Import python modules
import rospy as rp
import baxter_interface as bax
import time as t
#import thread
#import os
from baxter_interface import Gripper

#def disable_gravity():
#    os.execv('/opt/ros/indigo/bin/rostopic', ['rostopic', 'pub', '-r', '10',
#    '/robot/limb/left/suppress_gravity_compensation', 'std_msgs/Empty'])

# u: force
def apply_force(u):
    limb.set_joint_velocities(u)


def control(q):
    #pub = rp.Publisher('/robot/limb/left/suppress_gravity_compensation',
    #                   std_msgs.String)
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
    limb.move_to_joint_positions(startpos_4)

if __name__ == "__main__":
    # Initialise such that it registers somehow or something, I don't know...
    rp.init_node('Hello_Baxter')

    # Instance of baxters Limb class
    limb = bax.Limb('left')

    # Get joint startpos_1
    startpos_1 = dict()
    startpos_2 = dict()
    startpos_3 = dict()
    startpos_4 = dict()

    # Get gripper
    left_gripper = Gripper('left')

    # Define start positions
    startpos_1['left_s0'] =  1.7015681889618952
    startpos_1['left_s1'] = -0.6093738679874806
    startpos_1['left_e0'] = -0.9955535313376335
    startpos_1['left_e1'] =  1.2640001692175808
    startpos_1['left_w0'] = -0.5694903675024598
    startpos_1['left_w1'] =  1.3418496942027656
    startpos_1['left_w2'] = -1.4519128157335441

    startpos_2['left_s0'] =  1.2210487071567893
    startpos_2['left_s1'] = -0.19251458887961942
    startpos_2['left_e0'] =  0.02684466378799474
    startpos_2['left_e1'] =  2.0175682312662904
    startpos_2['left_w0'] = -1.1075341288532687
    startpos_2['left_w1'] =  1.499466220157992
    startpos_2['left_w2'] = -2.9686363197552468

    startpos_3['left_s0'] = -0.744364177321397 
    startpos_3['left_s1'] =  0.6469563972906732
    startpos_3['left_e0'] = -0.2462039164556089
    startpos_3['left_e1'] = -0.0502378708032473
    startpos_3['left_w0'] = -3.0418839023767754
    startpos_3['left_w1'] =  0.24505343086469483
    startpos_3['left_w2'] = -3.0583741958465436
    #startpos_3['left_e1'] = 1.0
    #startpos_3['left_w1'] = 1.0

    startpos_4['left_w0'] = -3.0426508927707183
    startpos_4['left_w1'] = -0.6181942575178218
    startpos_4['left_w2'] = -0.1917475984856767
    startpos_4['left_e0'] = -2.9786071948765023
    startpos_4['left_e1'] = -0.05062136600021865
    startpos_4['left_s0'] = -0.5971020216843973
    startpos_4['left_s1'] =  1.0465583925348236

    startpos_5['left_w0'] = -0.7834806874124751
    startpos_5['left_w1'] = -0.3857961681531816
    startpos_5['left_w2'] = -2.6349954983901696
    startpos_5['left_e0'] = -2.5713352956929247
    startpos_5['left_e1'] =  0.25732527716777814
    startpos_5['left_s0'] = -0.45674277959288195
    startpos_5['left_s1'] =  1.04272344056511


    # Move right arm to those positions
    limb.move_to_joint_positions(startpos_5)
    #left_gripper.command_position(100)
    print("Gripper open")
    #left_gripper.command_position(50)
    print("Look out")
    #control(1)

    print limb.joint_angles()

    print('All done')

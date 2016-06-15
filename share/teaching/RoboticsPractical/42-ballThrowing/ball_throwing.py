#! /usr/bin/env python2
# Import python modules
import rospy as rp
import baxter_interface as bax
import time as t
from baxter_interface import Gripper





if __name__ == "__main__":
    # Initialise such that it registers somehow or something, I don't know...
    rp.init_node('Hello_Baxter')

    # Instance of baxters Limb class
    limb = bax.Limb('left')

    # Get joint startpos_1
    startpos_1 = {}
    startpos_2 = {}

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

    # Move right arm to those positions
    limb.move_to_joint_positions(startpos_1)
    left_gripper.command_position(90)
    t.sleep(2)
    left_gripper.command_position(50)

    print('All done')

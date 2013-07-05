#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')

import rospy
import smach
import smach_ros

import the_curious_robot.msg as msgs
import require_provide as rp

import threading
#TODO: import ActionServer

class StateMachine:
    def __init__(self):
        rospy.init_node('tcr_state_machine')

        self.sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        
        with self.sm:
            smach.StateMachine.add(
                    'INIT', smach_ros.SimpleActionState('init',
                        msgs.PickOOIAction),
                    transitions={'succeeded':'PICK_OOI',
                        'aborted':'INIT'}
            )
            smach.StateMachine.add(
                    'PICK_OOI', smach_ros.SimpleActionState('pick_ooi',
                        msgs.PickOOIAction),
                    transitions={'succeeded':'GOTO_OOI', 
                        'aborted':'PICK_OOI'}
            )
            smach.StateMachine.add(
                    'GOTO_OOI', smach_ros.SimpleActionState('goto_ooi',
                        msgs.GotoOOIAction),
                    #transitions={'succeeded':'GOTO_OOI', 
                        #'aborted':'GOTO_OOI'}
                    transitions={'succeeded':'ARTICULATE_OOI', 
                        'aborted':'ARTICULATE_OOI'}
            )
            smach.StateMachine.add(
                    'ARTICULATE_OOI', smach_ros.SimpleActionState('articulate_ooi',
                        msgs.ArticulateOOIAction),
                    transitions={'succeeded':'OBSERVE_OOI_TRAJ', 
                        'aborted':'PICK_OOI'}
            )
            smach.StateMachine.add(
                    'OBSERVE_OOI_TRAJ',
                    smach_ros.SimpleActionState('observe_ooi_traj',
                        msgs.ObserveOOITrajAction),
                    transitions={'succeeded':'LEARN', 
                        'aborted':'LEARN'}
            )
            smach.StateMachine.add(
                    'LEARN',
                    smach_ros.SimpleActionState('learn',
                        msgs.LearnAction),
                    transitions={'succeeded':'PICK_OOI', 
                        'aborted':'PICK_OOI'}
            )

        self.sis = smach_ros.IntrospectionServer('tcr_sis_server', self.sm, '/SM_ROOT')
        self.sis.start()

        rp.Require(["Controller", "Perception"])

    def run(self):
        smach_ros.util.set_preempt_handler(self.sm)

        smach_thread = threading.Thread(target=self.sm.execute)
        smach_thread.start()
        
        rospy.spin()

        self.sis.stop()


if __name__ == '__main__':
    sm = StateMachine()
    sm.run()

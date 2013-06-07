#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')

import rospy
import smach
import smach_ros

import the_curious_robot.msg as msgs

#TODO: import ActionServer

def main():
    rospy.init_node('tcr_state_machine', log_level=rospy.DEBUG)

    sm = smach.StateMachine(outcomes=['succeeded'])
    
    with sm:
        smach.StateMachine.add(
                'INIT', smach_ros.SimpleActionState('init',
                    msgs.PickOOIAction),
                transitions={'succeeded':'PICK_OOI', 'preempted':'INIT',
                    'aborted':'INIT'}
        )
        smach.StateMachine.add(
                'PICK_OOI', smach_ros.SimpleActionState('pick_ooi',
                    msgs.PickOOIAction),
                transitions={'succeeded':'GOTO_OOI', 'preempted':'GOTO_OOI',
                    'aborted':'PICK_OOI'}
        )
        smach.StateMachine.add(
                'GOTO_OOI', smach_ros.SimpleActionState('goto_ooi',
                    msgs.GotoOOIAction),
                transitions={'succeeded':'ARTICULATE_OOI', 
                    'preempted':'ARTICULATE_OOI',
                    'aborted':'ARTICULATE_OOI'}
        )
        smach.StateMachine.add(
                'ARTICULATE_OOI', smach_ros.SimpleActionState('articulate_ooi',
                    msgs.ArticulateOOIAction),
                transitions={'succeeded':'OBSERVE_OOI_TRAJ', 
                    'preempted':'OBSERVE_OOI_TRAJ',
                    'aborted':'PICK_OOI'}
        )
        smach.StateMachine.add(
                'OBSERVE_OOI_TRAJ',
                smach_ros.SimpleActionState('observe_ooi_traj',
                    msgs.ObserveOOITrajAction),
                transitions={'succeeded':'LEARN', 
                    'preempted':'LEARN',
                    'aborted':'LEARN'}
        )
        smach.StateMachine.add(
                'LEARN',
                smach_ros.SimpleActionState('learn',
                    msgs.LearnAction),
                transitions={'succeeded':'PICK_OOI', 
                    'preempted':'PICK_OOI',
                    'aborted':'PICK_OOI'}
        )

    sis = smach_ros.IntrospectionServer('tcr_sis_server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()

if __name__ == '__main__':
    main()

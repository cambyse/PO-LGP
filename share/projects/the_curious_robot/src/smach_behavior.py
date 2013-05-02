#!/usr/bin/env python

import roslib
import rospy
roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import smach
import smach_ros

import copy

import orspy as ors
import random

import the_curious_robot.msg as msgs
from geometry_msgs.msg import Pose, Point, Quaternion


class Gaussian():
    """
    Represent Gaussians with this class.
    TODO maybe there is a Gaussian class that we can recycle.
    """
    def __init__(self, mu=0, sigma=99999):
        self.mu = mu
        self.sigma = sigma


class Properties():
    def __init__(self):
        self.joint = Gaussian()
        self.friction = Gaussian()
        self.weight = Gaussian()
        self.limit_min = Gaussian()
        self.limit_max = Gaussian()


class ObserveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['world_change', 'no_world_change'],
                output_keys=['trajectory'])
        self.world_belief = None
        self.world_changed = False
        self.trajectory = list()
        self.perception_sub = rospy.Subscriber(
            name='perception_updates',
            data_class=msgs.percept,
            callback=self.percept_cb)
        

    def percept_cb(self, data):
        # save the trajectory if somithing changed
        #print data
        if data.changed:
            rospy.logdebug("received perception msg with changed data")
            self.world_changed = True
            # No belief yet. Initialize belief with first messages
            if self.world_belief is None:
                rospy.loginfo("initializing world belief")
                self.objects_of_interest = []
                self.world_belief = ors.Graph()

                for body_str in data.bodies:
                    body = parse_body_msg(body_str)
                    self.world_belief.addObject(body)
                    self.objects_of_interest.append((body, Properties()))

            # otherwise log trajectory
            else:
                for body_str in data.bodies:
                    body = parse_body_msg(body_str)
                    pos = body.X.pos
                    self.trajectory.append(pos)
                    self.world_belief.getBodyByName(body.name).X = body.X

            self.world_belief.calcShapeFramesFromBodies()
            #print self.world_belief

        else:
            self.world_changed = False

    def execute(self, userdata):
        if self.world_changed:
            return 'world_change'
        else:
            #userdata.trajectory = copy.deepcopy(self.trajectory)  # clone!
            del self.trajectory[:]
            return 'no_world_change'

def parse_body_msg(msg):
    body = ors.Body()
    body_list = msg.split(' ', 1)
    body.name = body_list[0]
    body.read(body_list[1]) 
    return body

class LearnState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['learned'])

    def execute(self, userdata):
        rospy.loginfo('LEARN!')
        return 'learned'

class MoveState(smach.State):
    def __init__(self, observer):
        smach.State.__init__(self, outcomes=['moving','not_initialized'])
        self.control_pub = rospy.Publisher('control', msgs.control)
        self.observer = observer

    def get_best_target(self, objects_of_interest):
        object_of_interest = random.choice(objects_of_interest)
        pos = object_of_interest[0].com
        rot = object_of_interest[0].X.rot
        msg = msgs.control()
        msg.pose = Pose(Point(pos.x, pos.y, 1),
                    Quaternion(rot.x, rot.y, rot.z, rot.w))
        return msg

    def execute(self, userdata):
        if self.observer.world_belief is None:
            rospy.sleep(1)
            return 'not_initialized'
        target = self.get_best_target(self.observer.objects_of_interest)
        self.control_pub.publish(target)
        return 'moving'

class WaitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_arrived', 'arrived'])
        self.control_done_sub = rospy.Subscriber(
                name='control_done',
                data_class=msgs.control_done,
                callback=self.control_done_cb)
        self.active = False
        self.arrived = False

    def control_done_cb(self, data):
        rospy.logdebug('control done')
        self.arrived = True

    def execute(self, userdata):
        if self.arrived:
            self.arrived = False
            return 'arrived'
        else:
            rospy.sleep(1)
            return 'not_arrived'


def main():
    rospy.init_node('tcr_behavior', log_level=rospy.ERROR)

    sm = smach.StateMachine(outcomes=['arrived'])
    observer = ObserveState()

    with sm:
        smach.StateMachine.add('OBSERVE', observer,
                transitions={'world_change':'OBSERVE',
                             'no_world_change':'LEARN'})
        smach.StateMachine.add('LEARN', LearnState(),
                transitions={'learned':'MOVE'})
        smach.StateMachine.add('MOVE', MoveState(observer),
                transitions={'moving':'WAIT',
                             'not_initialized':'MOVE'})
        smach.StateMachine.add('WAIT', WaitState(),
                transitions={'not_arrived':'WAIT',
                             'arrived':'OBSERVE'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()

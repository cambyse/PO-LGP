#!/usr/bin/env python

# ROS imports
import roslib
import rospy
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import smach
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion
import the_curious_robot.msg as msgs
from util import Properties

import the_curious_robot.msg as msgs

from articulation_msgs.srv import TrackModelSrv, TrackModelSrvRequest
from articulation_msgs.msg import ModelMsg, TrackMsg

# python imports
# import copy
import numpy as np
import random

# MLR imports
import orspy as ors


###############################################################################
# different joint types
PRISMATIC = 0
ROTATIONAL = 1
MODELS = {
    PRISMATIC: 'prismatic',
    ROTATIONAL: 'rotational'
}


def get_trajectory(model=PRISMATIC, n=100, noise=0.02):
    """
    Helper function to create a trajectory for the given `model` with `n`
    samples and the noise of `noise`.
    """
    if model not in MODELS:
        raise NameError("unknown model, cannot generate trajectory!")

    msg = TrackMsg()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "/"
    msg.id = model

    for i in range(0, n):
        q = i / float(n)
        if model == PRISMATIC:
            pose = Pose(Point(q, 0, 0),
                        Quaternion(0, 0, 0, 1))
        elif model == ROTATIONAL:
            pose = Pose(Point(np.sin(q), np.cos(q) - 1.0, 0),
                        Quaternion(0, 0, 0, 1))
        # add noise
        pose.position.x += np.random.rand() * noise
        pose.position.y += np.random.rand() * noise
        pose.position.z += np.random.rand() * noise

        msg.pose.append(pose)
    return msg


def parse_body_msg(msg):
    body = ors.Body()
    body_list = msg.split(' ', 1)
    body.name = body_list[0]
    body.read(body_list[1])
    return body


###############################################################################
# STATES
###############################################################################
class ObserveState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['world_change', 'no_world_change'],
                             input_keys=['object_of_interest'],
                             output_keys=['trajectory'])
        self.world_belief = None
        self.object_of_interest = None
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
            #rospy.logdebug("received perception msg with changed data")
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
                    if body.name != self.object_of_interest:
                        continue
                    pos = body.X.pos
                    self.trajectory.append(pos)
                    self.world_belief.getBodyByName(body.name).X = body.X

            self.world_belief.calcShapeFramesFromBodies()
            #print self.world_belief

        else:
            self.world_changed = False

    def execute(self, userdata):
        self.object_of_interest = userdata.object_of_interest
        if self.world_changed:
            return 'world_change'
        else:
            userdata.trajectory = self.trajectory  # TODO: clone!
            rospy.logdebug(self.object_of_interest)
            rospy.logdebug(self.trajectory)

            del self.trajectory[:]
            return 'no_world_change'


###############################################################################
class LearnState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['learned'],
                             input_keys=['trajectory'])
        self.dof_learner = rospy.ServiceProxy('model_select', TrackModelSrv)
        self.model_pub = rospy.Publisher('model', ModelMsg)

    def execute(self, userdata):
        rospy.loginfo('LEARN!')
        self.learn_dof(userdata.trajectory)
        return 'learned'

    def learn_dof(self, trajectory=None):
        """ Learn DOF """
        # test every model via the service and publish the result
        for model_type, model_name in MODELS.items():
            request = TrackModelSrvRequest()
            # TODO replcace get_trajectory() with the percepts
            request.model.track = get_trajectory(model_type)
            try:
                # here we learn
                response = self.dof_learner(request)
                logLH = [entry.value for entry in response.model.params
                         if entry.name == 'loglikelihood'][0]
                print "selected model: '%s' (n = %d, log LH = %f)" % (
                    response.model.name,
                    len(response.model.track.pose),
                    logLH
                )
                self.model_pub.publish(response.model)
            except rospy.ServiceException:
                print "model selection failed"
            if rospy.is_shutdown():
                exit(0)
            print
            rospy.sleep(0.5)


class MoveState(smach.State):
    def __init__(self, observer):
        smach.State.__init__(self, outcomes=['moving', 'not_initialized'],
                output_keys=['object_of_interest'])
        self.control_pub = rospy.Publisher('control', msgs.control)
        self.observer = observer

    def get_best_target(self, objects_of_interest):
        object_of_interest = random.choice(objects_of_interest)
        pos = object_of_interest[0].X.pos
        rot = object_of_interest[0].X.rot
        msg = msgs.control()
        msg.pose = Pose(Point(pos.x, pos.y, 1),
                        Quaternion(rot.x, rot.y, rot.z, rot.w))
        return (object_of_interest[0].name, msg)

    def execute(self, userdata):
        if self.observer.world_belief is None:
            rospy.sleep(1)
            return 'not_initialized'
        target, msg = self.get_best_target(self.observer.objects_of_interest)
        userdata.object_of_interest = target
        self.control_pub.publish(msg)
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


###############################################################################
def shut_up(msg):
    pass


def main():
    rospy.init_node('tcr_behavior', log_level=rospy.ERROR)

    sm = smach.StateMachine(outcomes=['arrived'])
    sm.userdata.sm_object_of_interest = None

    observer = ObserveState()

    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    #sis.start()

    with sm:
        # smach.loginfo = shut_up
        smach.StateMachine.add(
            'OBSERVE', observer,
            transitions={'world_change': 'OBSERVE', 'no_world_change': 'LEARN'},
            remapping={'object_of_interest':'sm_object_of_interest'}
        )
        smach.StateMachine.add(
            'LEARN', LearnState(),
            transitions={'learned': 'MOVE'}
        )
        smach.StateMachine.add(
            'MOVE', MoveState(observer),
            transitions={'moving': 'WAIT', 'not_initialized': 'MOVE'},
            remapping={'object_of_interest':'sm_object_of_interest'}
        )
        smach.StateMachine.add(
            'WAIT', WaitState(),
            transitions={'not_arrived': 'WAIT', 'arrived': 'OBSERVE'}
        )

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('tcr_sis_server', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()
    print outcome

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

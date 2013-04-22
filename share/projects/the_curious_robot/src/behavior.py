#!/usr/bin/env python

"""
The behavior of the curious robot.

NOTE: this does not work yet!
"""

import roslib
roslib.load_manifest('the_curious_robot')
import rospy
import numpy as np
import random

from articulation_msgs.msg import ModelMsg, TrackMsg
from articulation_msgs.srv import TrackModelSrv, TrackModelSrvRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import the_curious_robot.msg as msgs
# from sensor_msgs.msg import ChannelFloat32
# from articulation_msgs.msg import *
# from articulation_msgs.srv import *
import orspy as ors


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


class Behavior():
    """
    The actual behavior of the robot.
    """
    def __init__(self):
        # init the node: test_fitting
        rospy.init_node('tcr_behavior')

        # pub and sub
        self.dof_learner = rospy.ServiceProxy('model_select', TrackModelSrv)
        # TODO we don't really need the publisher.
        # TODO visualize it with ors.
        self.model_pub = rospy.Publisher('model', ModelMsg)
        self.control_pub = rospy.Publisher('control', msgs.control)
        self.perception_sub = rospy.Subscriber(
            name='perception_updates',
            data_class=msgs.percept,
            callback=self.percept_cb
        )

        # this is funny and true
        self.world_belief = None

        self.percepts = None
        self.trajectory = []

    def run(self):
        """ the behavior loop """
        while True:
            self.step()

    def step(self):
        # tmp
        self.learn_dof()
        msg = msgs.control()
        # move randomly
        msg.pose.position.x = random.randint(-5, 5)
        msg.pose.position.y = random.randint(-5, 5)
        #msg.pose.position.z = random.randint(-5,5)
        self.control_pub.publish(msg)
        return

        # Here is the actual behavior
        # reset control --> don't move
        msg.pose.position.x = 0.
        msg.pose.position.y = 0.
        msg.pose.position.z = 0.

        if self.percepts.world_changed:
            self.trajectory.append(self.percepts.change)
            self.prev_change = True

        elif self.prev_change:
            self.learn_dof(self.trajectory)
            # TODO add learnend DOF to belief
            # TODO explore DOF
            # TODO learn more about DOF
            self.learn_limits_of_dof()

            self.prev_change = False
            del self.trajectory[:]

        else:
            # TODO general exploration
            # move randomly
            msg.pose.position.x = random.randint(-5, 5)
            msg.pose.position.y = random.randint(-5, 5)
            #msg.pose.position.z = random.randint(-5,5)

        self.control_pub.publish(msg)

    def learn_dof(self, trajectory=None):
        """
        Learn DOF
        """
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

    def learn_limits_of_dof(self):
        # TODO these are not the limits of the joint but he min/max pose
        #      but the joint value can be inferred
        print "learning limits:"
        print "  min x pose:", min(self.trajectory, key=lambda pose: pose.x)
        print "  min y pose:", min(self.trajectory, key=lambda pose: pose.y)
        print "  min z pose:", min(self.trajectory, key=lambda pose: pose.z)
        print "  man x pose:", max(self.trajectory, key=lambda pose: pose.x)
        print "  man y pose:", max(self.trajectory, key=lambda pose: pose.y)
        print "  man z pose:", max(self.trajectory, key=lambda pose: pose.z)

    def percept_cb(self, data):
        self.percepts = data


if __name__ == '__main__':
    behavior = Behavior()
    behavior.run()

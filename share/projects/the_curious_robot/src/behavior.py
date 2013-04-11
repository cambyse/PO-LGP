#!/usr/bin/env python

"""
The behavior of the curious robot.

NOTE: this does not work yet!
"""

import roslib
roslib.load_manifest('the_curious_robot')
import rospy
import numpy as np

from articulation_msgs.msg import ModelMsg, TrackMsg
from articulation_msgs.srv import TrackModelSrv, TrackModelSrvRequest
from geometry_msgs.msg import Pose, Point, Quaternion
# from sensor_msgs.msg import ChannelFloat32
# from articulation_msgs.msg import *
# from articulation_msgs.srv import *


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

        # this is funny and true
        self.world_belief = None

    def run(self):
        """ the behavior loop """
        while True:
            self.step()

    def step(self):
        self.learn_dof()
        return

        # TODO this should happen at some point
        if percepts.changed:
            self.learn_dof()
            # TODO add learnend DOF to belief
            # TODO explore DOF
            # TODO learn more about DOF
            # self.learn_limits_of_dof()
        else:
            # TODO general exploration
            pass

    def learn_dof(self):
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
        pass


if __name__ == '__main__':
    behavior = Behavior()
    behavior.run()

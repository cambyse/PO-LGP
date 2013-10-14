#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer

import corepy
import orspy
import the_curious_robot.msg as msgs
from articulation_msgs.msg import ModelMsg, TrackMsg
from articulation_msgs.srv import TrackModelSrv, TrackModelSrvRequest
import util
import require_provide as rp

# different joint types
PRISMATIC = 0
ROTATIONAL = 1
MODELS = {
    PRISMATIC: 'prismatic',
    ROTATIONAL: 'rotational'
}


class LearnActionServer:

    def __init__(self, name):
        # subscriber
        self.trajectory_sub = rospy.Subscriber(
            "ooi_trajectory", msgs.Trajectory, self.trajectory_cb
        )

        # publisher
        self.world_belief_pub = rospy.Publisher('world_belief', msgs.ors)

        # services
        self.dof_learner = rospy.ServiceProxy('model_select', TrackModelSrv)

        # real member
        self.trajectory = []

        # action server
        self.server = SimpleActionServer(
            name, msgs.LearnAction, execute_cb=self.execute, auto_start=False)
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        # Belief & PhysX & OpenGL
        self.belief = orspy.Graph()
        self.gl = corepy.OpenGL()
        self.physx = orspy.PhysXInterface()
        orspy.bindOrsToPhysX(self.belief, self.gl, self.physx)

        self.learned_bodies = []
        self.learned_shapes = []

        # require/provide
        rp.Provide("Learn")

    def execute(self, msg):
        if not self.trajectory:
            self.server.set_aborted()
            return

        request = TrackModelSrvRequest()
        request.model.track = util.create_track_msg(self.trajectory)

        try:
            # here we learn
            response = self.dof_learner(request)

            # rospy.loginfo(response.model)

            if response.model.params:
                print response

                logLH = [entry.value for entry in response.model.params
                         if entry.name == 'loglikelihood'][0]
                rospy.loginfo("selected model: '%s' (n = %d, log LH = %f)" % (
                    response.model.name,
                    len(response.model.track.pose),
                    logLH
                ))
            # TODO: add to world_belief and publish
        except rospy.ServiceException:
            self.server.set_aborted()
        self.server.set_succeeded()

    def preempt_cb(self):
        self.server.set_preempted()

    def trajectory_cb(self, msg):
        del self.trajectory[:]
        self.trajectory = []
        self.ooi, self.trajectory = util.parse_trajectory_msg(msg)


def main():
    rospy.init_node('tcr_sas_learn')
    server = LearnActionServer('learn')


if __name__ == '__main__':
    main()
    rospy.spin()

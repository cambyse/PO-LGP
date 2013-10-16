#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer

import corepy
import orspy
import the_curious_robot.msg as msgs
#from articulation_msgs.msg import ModelMsg, TrackMsg
from articulation_msgs.srv import TrackModelSrv, TrackModelSrvRequest

import util
from util import ObjectTypeHypo

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

        # action server
        self.server = SimpleActionServer(
            name, msgs.LearnAction, execute_cb=self.execute, auto_start=False)
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        # real member
        self.ooi = None
        self.trajectory = []

        # Belief & PhysX & OpenGL
        self.belief = orspy.Graph()
        self.gl = corepy.OpenGL()
        self.physx = orspy.PhysXInterface()
        orspy.bindOrsToPhysX(self.belief, self.gl, self.physx)

        self.learned_bodies = []
        self.learned_shapes = []

        # save all learned object in here
        self.learned_tmp = {}

        # require/provide
        rp.Provide("Learn")

    def execute(self, msg):
        print msg

        # add object hypothesis if it does not exist yet
        if self.ooi not in self.learned_tmp:
            self.learned_tmp[self.ooi] = ObjectTypeHypo()

        # no trajectory observed --> static object
        if not self.trajectory:
            rospy.loginfo("updating with static object")
            self.learned_tmp[self.ooi].update(ObjectTypeHypo.STATIC)
            self.server.set_succeeded()

        # trajectory observed --> classify joint
        else:
            rospy.logwarn("CAN'T learn joints yet")
            self.learned_tmp[self.ooi].update(ObjectTypeHypo.FREE)
            self.server.set_succeeded()

        for name, hypo in self.learned_tmp.iteritems():
            print "{}: {}".format(name, hypo)

        return

        self.learned_bodies.append(orspy.Body(self.belief))
        body = self.learned_bodies[-1]

        body.X.pos.setRandom()
        body.X.pos.z += 1
        body.name = "body_" + str(len(self.learned_bodies))

        self.learned_shapes.append(orspy.Shape(self.belief, body))
        shape = self.learned_shapes[-1]
        shape.type = orspy.sphereST
        shape.set_size(.1, .1, .1, .1)

        self.belief.calcShapeFramesFromBodies()
        self.gl.update()
        print
        print "adding objects -- #%d" % self.belief.bodies.N
        print
        return
        request = TrackModelSrvRequest()
        request.model.track = util.create_track_msg(self.trajectory)

        rospy.loginfo(self.belief.bodies.N)

        try:
            # here we learn
            response = self.dof_learner(request)
            # rospy.loginfo(response.model)
            if response.model.params:
                # print response
                # print response.model.name

                for p in [p for p in response.model.params
                          if p.name.startswith("rot")]:
                    # print p
                    if p.name == "rot_center.x":
                        # print "rot_center.x"
                        x = p.value
                    if p.name == "rot_center.y":
                        # print "rot_center.y"
                        y = p.value
                    if p.name == "rot_center.z":
                        # print "rot_center.z"
                        z = p.value
                try:
                    print x, y, z
                    self.learned_bodies.append(orspy.Body(self.belief))
                    body = self.learned_bodies[-1]

                    body.X.pos.setRandom()
                    body.X.pos.z += 1
                    body.name = "body_" + str(len(self.learned_bodies))

                    self.learned_shapes.append(orspy.Shape(self.belief, body))
                    shape = self.learned_shapes[-1]
                    shape.type = orspy.sphereST
                    shape.set_size(.1, .1, .1, .1)

                    self.belief.calcShapeFramesFromBodies()
                    self.gl.update()

                except Exception:
                    print
                    print "rot center not set"
                    print

                logLH = [entry.value
                         for entry in response.model.params
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

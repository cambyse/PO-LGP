#!/usr/bin/env python
# encoding: utf-8

# ROS
import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer
# MLR
import orspy
import guipy
import rosors
from rosors import parser
import rosors.srv
# import corepy
import the_curious_robot.msg as msgs
from articulation_msgs.srv import TrackModelSrv, TrackModelSrvRequest
import util
import pickle_logger
import belief_representations as bel_rep
from belief_representations import ObjectTypeHypo, JointBelief
import require_provide as rp
from timer import Timer
# python std
import numpy as np


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
        self.request_shapes = rospy.ServiceProxy('/world/shapes',
                                                 rosors.srv.Shapes)

        # action server
        self.server = SimpleActionServer(name,
                                         msgs.LearnAction,
                                         execute_cb=self.execute,
                                         auto_start=False)
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        # real member
        self.ooi = None
        self.trajectory = []

        # Belief in ors
        self.belief_ors = orspy.Graph()
        # The BeliefAnnotation is the probabilistic counterpart to the ors
        # graph/belief representation.
        self.belief_annotation = bel_rep.Annotation()
        self._added_bodies = []
        self._added_shapes = []

        # PhysX & OpenGL of belief
        self.gl = guipy.OpenGL()
        # self.physx = orspy.PhysXInterface()
        # orspy.bindOrsToPhysX(self.belief, self.gl, self.physx)
        orspy.bindOrsToOpenGL(self.belief_ors, self.gl)

        # require/provide
        rp.Provide("Learn")

    @pickle_logger.pickle_member("belief_annotation")
    def execute(self, msg):
        # add object to belief and belief_annotation if it does not exist yet
        if self.ooi not in self.belief_annotation:
            with Timer("Adding new shape with id %d", rospy.loginfo):
                shape_response = self.request_shapes(index=self.ooi,
                                                     with_mesh=True)
                shape_msg = shape_response.shapes[0]
                print shape_msg

                # add shape and body to belief
                self._added_bodies.append(orspy.Body(self.belief_ors))
                body = self._added_bodies[-1]
                self._added_shapes.append(orspy.Shape(self.belief_ors, body))
                shape = self._added_shapes[-1]

                self.belief_annotation[self.ooi] = bel_rep.ShapeBelief(shape)

                # Set BODY infos
                body.pose = parser.ros_to_ors_transform(shape_msg.X,
                                                        shape_msg.Xvel)

                # Set SHAPE infos
                shape.type = shape_msg.shape_type
                shape.set_size(shape_msg.size[0], shape_msg.size[1],
                               shape_msg.size[2], shape_msg.size[3])
                if shape.type == orspy.meshST and shape_msg.mesh is not None:
                    shape.mesh = parser.msg_to_ors_mesh(shape_msg.mesh)

                shape.X = parser.ros_to_ors_transform(shape_msg.X,
                                                      shape_msg.Xvel)
                shape.rel = parser.ros_to_ors_transform(shape_msg.rel,
                                                        shape_msg.relvel)

        #######################################################################
        # Belief update
        #######################################################################
        # This consists of two parts:
        #  - updating all parts of the belief annotation
        #  - TODO then transfer all information from the annotation to the
        #    belief_ors
        shape_anno = self.belief_annotation[self.ooi]
        shape = shape_anno.belief_shape

        # Update ObjectTypeHypo
        shape_anno.object_type.update(ObjectTypeHypo.STATIC
                                      if len(self.trajectory) == 0 else
                                      ObjectTypeHypo.FREE)

        # Update JointInformation
        if len(self.trajectory) > 1:
            rospy.loginfo("Learning DoF")
            self.update_dof(shape_anno)
            # self.update_dynamics()

        # PRINT
        rospy.loginfo(str(self.belief_annotation))

        # VISUALIZE
        self.visualize_object_type()
        # self.visualize_object_entropy()

        self.gl.update()
        self.server.set_succeeded()

    def visualize_object_type(self):
        """
        Colorize the shapes depending on the ObjectTypeHypo.
        """
        for shape_anno in self.belief_annotation.itervalues():
            shape = shape_anno.belief_shape

            if shape_anno.object_type.is_static():
                shape.set_color(0., 0., 0.)
            else:
                shape.set_color(1., 1., 1.)

    def visualize_object_entropy(self):
        # use entropy to visualize the scene.
        # normalize entropy to be between 0 - 1
        entropies = {k: (shape.object_type.get_entropy(), shape)
                     for k, shape in
                     self.belief_annotation.iteritems()}
        min_entropy = min(entropy for (entropy, _) in entropies.values())
        max_entropy = max(entropy for (entropy, _) in entropies.values())
        diff = max_entropy - min_entropy
        # print "min:", min_entropy, " max:", max_entropy, " diff:", diff
        for k, (entropy, shape) in entropies.iteritems():
            color = (entropy - min_entropy) / diff

            # set color according to the entropy
            self.getShapeById(shape.belief_shape_id).set_color(
                color, color, color)

    def update_dof(self, shape_anno):
        request = TrackModelSrvRequest()
        request.model.track = util.create_track_msg(self.trajectory)

        # here we learn
        response = self.dof_learner(request)
        # rospy.loginfo(response)

        # stop if it's not an rotational or prismatic joint
        if response.model.name not in ["rotational", "prismatic"]:
            return

        rospy.loginfo("joint classified as, %s", response.model.name)

        # make sure we have an JointBelief instance
        if shape_anno.joint is None:
            shape_anno.joint = JointBelief()

        # update common stuff
        loglikelihood = filter(lambda param: param.name == 'loglikelihood',
                               response.model.params)[0]
        shape_anno.joint.loglikelihood = loglikelihood
        # save all parameters in the belief
        for p in response.model.params:
            # if p.name.startswith("rot"):
            shape_anno.joint.values[p.name] = p.value

        if response.model.name == "rotational":
            shape_anno.joint.update(JointBelief.ROTATIONAL)
            # important information
            # - rot_center[3], points to the rotational center of the
            #   rotational joint
            # - rot_axis[4], gives the rotational axis of the rotational joint
            # - rot_radius, gives the radius of the rotational joint
            # - rot_orientation[4], gives the orientation of the articulated
            #   object

            # we might use this later
            # x = filter(lambda param: param.name == 'rot_center.x',
            #            response.model.params)[0].value
            # y = filter(lambda param: param.name == 'rot_center.y',
            #            response.model.params)[0].value
            # z = filter(lambda param: param.name == 'rot_center.z',
            #            response.model.params)[0].value
            # rots = filter(lambda p: p.name.startswith("rot"),
            #               response.model.params)

        elif response.model.name == "prismatic":
            shape_anno.joint.update(JointBelief.PRISMATIC)
            # The parameters of the prismatic model are:
            #
            # rigid_position[3], gives the average position of the articulated
            # object (inherited from the rigid model)
            #
            # rigid_orientation[4], gives the average orientation of the
            # articulated object (inherited from the rigid model)
            #
            # prismatic_dir[3], gives the line direction of the prismatic joint
            rospy.logerr("PRISMATIV evaluation not implemented yet")

        else:
            # The parameters of the rigid model are:
            # - rigid_position[3], gives the estimated position of the rigid
            #   object.
            # - rigid_orientation[4], gives the estimated orientation of the
            #   rigid object.  rigid_width, gives the estimated width of the
            #   rigid object (only if a width channel is present in
            #   ModelMsg::track.channels).
            # - rigid_height, gives the estimated height of the rigid object
            #   (only if a height channel is present in
            #   ModelMsg::track.channels).

            rospy.logerr("Joint type is not handled.")

    def preempt_cb(self):
        self.server.set_preempted()

    def trajectory_cb(self, msg):
        del self.trajectory[:]
        self.trajectory = []
        self.ooi, self.trajectory = util.parse_trajectory_msg(msg)

    def getShapeById(self, idx):
        """
        Return the shape with the given `id`.
        TODO this should be part of ors.
        """
        for shape in self.belief_ors.shapes:
            if shape.index == idx:
                return shape
        return None


def main():
    rospy.init_node('tcr_sas_learn')
    server = LearnActionServer('learn')
    rospy.spin()


if __name__ == '__main__':
    main()

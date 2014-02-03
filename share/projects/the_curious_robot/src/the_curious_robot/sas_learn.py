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

import the_curious_robot as tcr
import the_curious_robot.util
import the_curious_robot.msg as msgs
import the_curious_robot.belief_representations as bel_rep
from the_curious_robot import require_provide
from the_curious_robot import pickle_logger
from the_curious_robot.timer import Timer
from the_curious_robot import srv

from articulation_msgs.srv import TrackModelSrv, TrackModelSrvRequest
# python std
import sys


class LearnActionServer(object):
    """
    Learn from the given observation and build a rich model of the world.

    This class contains the belief (ors & probabilicit counterpart).
    """

    def __init__(self, name):
        # SUBSCRIBER
        self.trajectory_sub = rospy.Subscriber("ooi_trajectory",
                                               tcr.msg.Trajectory,
                                               self.trajectory_cb)
        # PUBLISHER
        # None

        # SERVICE PUBLISHER
        self.entropy_service = rospy.Service("/belief/entropy",
                                             tcr.srv.Entropy,
                                             self.handle_entropy_request)
        # SERVICE PROXIES
        self.dof_learner = rospy.ServiceProxy('model_select',
                                              TrackModelSrv)
        self.request_shape = rospy.ServiceProxy('/world/shapes',
                                                rosors.srv.Shapes)
        self.request_all_shapes = rospy.ServiceProxy('/world/shapes',
                                                     rosors.srv.Shapes)
        # action server
        self.server = SimpleActionServer(name,
                                         msgs.LearnAction,
                                         execute_cb=self.execute,
                                         auto_start=False)
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        # REAL DATA
        all_shapes_msg = self.request_all_shapes(with_mesh=False)
        # uninitialized_oois are interesting because we don't know anything
        # about them.
        self.uninitialized_oois = set(shape.index
                                      for shape in all_shapes_msg.shapes
                                      if shape.name not in ["base", "robot"])

        # real member
        self.ooi = None
        self.trajectory = []

        # Belief in ors
        self.belief_ors = orspy.Graph()
        # The BeliefAnnotation is the probabilistic counterpart to the ors
        # graph/belief representation.
        self.belief = bel_rep.Belief()
        self._added_bodies = []
        self._added_shapes = []

        # OpenGL window for belief
        self.gl = guipy.OpenGL()
        orspy.bindOrsToOpenGL(self.belief_ors, self.gl)

        # require/provide
        require_provide.Provide("Learn")

    @pickle_logger.pickle_member(name="belief",
                                 folder=rospy.get_param("strategy_name"))
    def execute(self, msg):
        # add object to belief and belief if it does not exist yet
        if self.ooi not in self.belief:
            with Timer("Adding new shape with id %d", rospy.loginfo):
                # rm ooi from list of uninitialized_oois
                self.uninitialized_oois -= set([self.ooi])

                shape_response = self.request_shape(index=self.ooi,
                                                    with_mesh=True)
                shape_msg = shape_response.shapes[0]
                # print shape_msg

                # add shape and body to belief
                self._added_bodies.append(orspy.Body(self.belief_ors))
                body = self._added_bodies[-1]
                self._added_shapes.append(orspy.Shape(self.belief_ors, body))
                ors_shape = self._added_shapes[-1]

                self.belief[self.ooi] = bel_rep.ObjectBel(ors_shape.name)
                self.belief[self.ooi].ors_shape = ors_shape

                # Set BODY infos
                body.pose = parser.ros_to_ors_transform(shape_msg.X,
                                                        shape_msg.Xvel)

                # Set SHAPE infos
                ors_shape.type = shape_msg.shape_type
                ors_shape.set_size(shape_msg.size[0], shape_msg.size[1],
                                   shape_msg.size[2], shape_msg.size[3])
                if ors_shape.type == orspy.meshST and shape_msg.mesh is not None:
                    ors_shape.mesh = parser.msg_to_ors_mesh(shape_msg.mesh)

                ors_shape.X = parser.ros_to_ors_transform(shape_msg.X,
                                                          shape_msg.Xvel)
                ors_shape.rel = parser.ros_to_ors_transform(shape_msg.rel,
                                                            shape_msg.relvel)

        #############################################################
        # Belief update
        #############################################################
        # This consists of two parts:
        #  - updating all parts of the belief annotation
        #  - TODO then transfer all information from the annotation to the
        #    belief_ors
        obj_bel = self.belief[self.ooi]
        ors_shape = obj_bel.ors_shape

        obj_bel.update("movable" if self.trajectory else "static")
        if self.trajectory:
            rospy.loginfo("Learning DoF")
            self.update_dof(obj_bel)

        # PRINT
        #rospy.loginfo(str(self.belief))

        # VISUALIZE
        # self.visualize_object_type()
        # self.visualize_object_entropy()

        self.gl.update()
        self.server.set_succeeded()

    def visualize_object_type(self):
        """
        Colorize the shapes depending on the ObjectTypeHypo.
        """
        for obj_bel in self.belief.itervalues():
            ors_shape = obj_bel.ors_shape

            if obj_bel.object_type.is_static():
                ors_shape.set_color(0., 0., 0.)
            else:
                ors_shape.set_color(1., 1., 1.)

    def visualize_object_entropy(self):
        """Colorize the shapes depending on the object_type entropy."""
        gentor = self.belief.iter_entropy_normalized("object_type")
        for shape, entropy in gentor:
            # print "ID:", shape.index, "H/color:", entropy
            shape.set_color(entropy, entropy, entropy)

    def update_dof(self, obj_bel):
        request = TrackModelSrvRequest()
        request.model.track = tcr.util.create_track_msg(self.trajectory)
        #print self.trajectory

        # here we learn
        response = self.dof_learner(request)
        #rospy.loginfo(response)

        # stop if it's not an rotational or prismatic joint
        if response.model.name not in ["rotational", "prismatic"]:
            return

        rospy.loginfo("joint classified as %s", response.model.name)

        # update common stuff
        loglikelihood = filter(lambda param: param.name == 'loglikelihood',
                               response.model.params)[0]
        # obj_bel.joint_bel.loglikelihood = loglikelihood
        # save all parameters in the belief
        # for p in response.model.params:
        #     # if p.name.startswith("rot"):
        #     obj_bel.joint_bel.values[p.name] = p.value

        if response.model.name == "rotational":
            obj_bel.joint_bel.update("rot", response)
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
            obj_bel.joint_bel.update("pris", response)
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
        self.ooi, self.trajectory = tcr.util.parse_trajectory_msg(msg)

    # def getShapeById(self, idx):
    #     """
    #     Return the shape with the given `id`.
    #     TODO this should be part of ors.
    #     """
    #     for shape in self.belief_ors.shapes:
    #         if shape.index == idx:
    #             return shape
    #     return None

    def handle_entropy_request(self, req):
        """
        Calculate the entropy denpending on the requested entropy type.
        """
        res = srv.EntropyResponse()

        # uninitialized shapes have a high entropy
        for shape_idx in self.uninitialized_oois:
            res.shape_ids.append(shape_idx)
            res.entropies.append(sys.float_info.max)

        # add the already explored shapes
        entropies = self.belief.get_entropy()
        for shape_id, entropy in entropies:
            res.shape_ids.append(shape_id)
            res.entropies.append(entropy)

        return res


def main():
    rospy.init_node('tcr_sas_learn')
    server = LearnActionServer('learn')
    rospy.spin()


if __name__ == '__main__':
    main()

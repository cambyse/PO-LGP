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
import rosors
import rosors.parser
import rosors.srv
# import corepy
import the_curious_robot.msg as msgs
from articulation_msgs.srv import TrackModelSrv, TrackModelSrvRequest
import util
import pickle_logger
import belief_representations as rep
from belief_representations import ObjectTypeHypo, JointBelief
import require_provide as rp
# python std
# import numpy as np


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

        # Belief
        self.belief = orspy.Graph()
        # The BeliefAnnotation is the probabilistic counterpart to the ors
        # graph/belief representation.
        # It's a mapping:  "shape_id" --> ShapeBelief
        self.belief_annotation = {}
        self._added_bodies = []
        self._added_shapes = []

        # PhysX & OpenGL of belief
        # self.gl = guipy.OpenGL()
        # self.physx = orspy.PhysXInterface()
        # orspy.bindOrsToPhysX(self.belief, self.gl, self.physx)
        # orspy.bindOrsToOpenGL(self.belief, self.gl)

        # require/provide
        rp.Provide("Learn")

    @pickle_logger.pickle_member("belief_annotation")
    def execute(self, msg):
        # add object to belief and belief_annotation if it does not exist yet
        if self.ooi not in self.belief_annotation:
            rospy.logwarn("TODO: adding new shape")
            # add shape and body to belief
            # self._added_bodies.append(orspy.Body(self.belief))
            # self._added_shapes.append(orspy.Shape(self.belief,
            #                                       self._added_bodies[-1]))
            # new_shape = self._added_shapes[-1]
            # # add to belief annotation
            self.belief_annotation[self.ooi] = rep.ShapeBelief(
                belief_shape_id=1  # self.belief.shapes[-1].index
            )

            # # fill body and shape with preliminary infor from the perception
            # shape_response = self.request_shapes(index=self.ooi)
            # shape_msg = shape_response.shapes[0]

            # new_shape.X = rosors.parser.ros_to_ors_transform(
            #     shape_msg.X, shape_msg.Xvel
            # )
            # new_shape.rel = rosors.parser.ros_to_ors_transform(
            #     shape_msg.rel, shape_msg.relvel
            # )
            # new_shape.type = shape_msg.shape_type
            # if new_shape.type == orspy.meshST and shape_msg.mesh:
            #     # c&p from parser: what causes the stupid crash?
            #     new_shape.mesh = guipy.Mesh()

            #     V = np.resize(new_shape.mesh.V,
            #                   [len(shape_msg.mesh.vertices), 3])
            #     for i in range(len(shape_msg.mesh.vertices)):
            #         v = shape_msg.mesh.vertices[i]
            #         V[i, 0] = v.x
            #         V[i, 1] = v.y
            #         V[i, 2] = v.z
            #     new_shape.mesh.V = V  # need to assign members, because of swig

            #     T = np.resize(new_shape.mesh.T,
            #                   [len(shape_msg.mesh.triangles), 3])
            #     for i in range(len(shape_msg.mesh.triangles)):
            #         t = shape_msg.mesh.triangles[i]
            #         T[i, 0] = t.vertex_indices[0]
            #         T[i, 1] = t.vertex_indices[1]
            #         T[i, 2] = t.vertex_indices[2]
            #     new_shape.mesh.T = T  # see above
            #     # new_shape.mesh.thisown = False
            # self.belief.calcShapeFramesFromBodies()

        #######################################################################
        # Belief update
        #######################################################################
        # This consists of two parts:
        #  - updating all parts of the belief annotation
        #  - TODO then transfer all information from the annotation to the
        #    belief
        shape_anno = self.belief_annotation[self.ooi]

        # Update ObjectTypeHypo
        shape_anno.object_type.update(ObjectTypeHypo.STATIC
                                      if len(self.trajectory) == 0 else
                                      ObjectTypeHypo.FREE)
        # Update JointInformation
        if len(self.trajectory) > 1:
            rospy.loginfo("Learning DoF")
            self.update_dof(shape_anno)
            # self.update_dynamics()

        rospy.loginfo(str(self.belief_annotation))
        self.server.set_succeeded()

    def update_dof(self, shape_anno):
        request = TrackModelSrvRequest()
        request.model.track = util.create_track_msg(self.trajectory)

        # here we learn
        response = self.dof_learner(request)
        rospy.loginfo(response)

        print response.model.name

        # stop if it's not an rotational or prismatic joint
        if response.model.name not in ["rotational", "prismatic"]:
            return

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

        elif response.name == "prismatic":
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
        print "getShapeById(", idx, ")"
        for shape in self.belief.shapes:
            print "looking at", shape.index
            if shape.index == idx:
                print "found"
                return shape
        return None


def main():
    rospy.init_node('tcr_sas_learn')
    server = LearnActionServer('learn')
    rospy.spin()


if __name__ == '__main__':
    main()

import unittest

import roslib
roslib.load_manifest("rosors")

from orspy import Shape, sphereST, meshST, Graph, Body, Joint
from corepy import Vector, Quaternion, Transformation
from guipy import Mesh

from rosors import parser

import geometry_msgs.msg
import shape_msgs.msg
import ors_msgs.msg


###############################################################################
# helper function for asserts
def assert_vector_equal(ros, ors):
    assert type(ros) == geometry_msgs.msg.Vector3
    assert type(ors) == Vector

    assert ros.x == ors.x
    assert ros.y == ors.y
    assert ros.z == ors.z


def assert_quaternion_equal(ros, ors):
    assert type(ros) == geometry_msgs.msg.Quaternion
    assert type(ors) == Quaternion

    assert ros.w == ors.w
    assert ros.x == ors.x
    assert ros.y == ors.y
    assert ros.z == ors.z


def assert_transform_equal(ros_transform, ros_twist, ors):
    assert type(ros_transform) == geometry_msgs.msg.Transform
    assert type(ros_twist) == geometry_msgs.msg.Twist
    assert type(ors) == Transformation

    assert_vector_equal(ros_transform.translation, ors.pos)
    assert_quaternion_equal(ros_transform.rotation, ors.rot)

    assert_vector_equal(ros_twist.linear, ors.vel)
    assert_vector_equal(ros_twist.angular, ors.angvel)


def assert_mesh_equal(ros, ors):
    assert type(ros) == shape_msgs.msg.Mesh
    assert type(ors) == Mesh

    for i in range(len(ros.vertices)):
        assert ors.V[i, 0] == ros.vertices[i].x
        assert ors.V[i, 1] == ros.vertices[i].y
        assert ors.V[i, 2] == ros.vertices[i].z
    for i in range(len(ros.triangles)):
        assert ors.T[i, 0] == ros.triangles[i].vertex_indices[0]
        assert ors.T[i, 1] == ros.triangles[i].vertex_indices[1]
        assert ors.T[i, 2] == ros.triangles[i].vertex_indices[2]


def assert_ors_mesh_equal(ors1, ors2):
    assert (ors1.V == ors2.V).all()
    assert (ors1.T == ors2.T).all()


def assert_shape_equal(ros, ors):
    assert ros.index == ors.index
    assert ros.index_body == ors.ibody
    assert ros.name == ors.name
    assert_transform_equal(ros.X, ros.Xvel, ors.X)
    assert_transform_equal(ros.rel, ros.relvel, ors.rel)
    assert ros.shape_type == ors.type
    assert ros.contact == ors.cont
    if ors.type == meshST or ros.shape_type == meshST:
        assert_mesh_equal(ros.mesh, ors.mesh)


def assert_ors_shape_equal(ors1, ors2, check_bodies_set=False):
    assert ors1.index == ors2.index
    assert ors1.ibody == ors2.ibody
    if check_bodies_set:
        assert ors1.body is not None
        assert ors2.body is not None
    if ors1.body is not None and ors2.body is not None:
        assert ors1.body.index == ors2.body.index
        assert ors1.ibody == ors1.body.index
        assert ors2.ibody == ors2.body.index
    assert ors1.name == ors2.name
    assert ors1.X == ors2.X
    assert ors1.rel == ors2.rel
    assert ors1.type == ors2.type
    assert ors1.cont == ors2.cont
    if ors1.type == meshST or ors2.type == meshST:
        assert_ors_mesh_equal(ors1.mesh, ors2.mesh)


def assert_body_equal(ros, ors):
    assert type(ros) == ors_msgs.msg.Body
    assert type(ors) == Body

    assert ros.index == ors.index
    assert ros.name == ors.name
    assert ros.mass == ors.mass
    assert ros.body_type == ors.type

    assert_transform_equal(ros.transform, ros.twist, ors.X)
    assert_vector_equal(ros.com, ors.com)
    assert_vector_equal(ros.force, ors.force)
    assert_vector_equal(ros.torque, ors.torque)

    for i, _ in enumerate(ros.shapes):
        assert_shape_equal(ros.shapes[i], ors.shapes[i])


def assert_ors_body_equal(ors1, ors2):
    assert ors1.index == ors2.index
    assert ors1.name == ors2.name
    assert ors1.mass == ors2.mass
    assert ors1.type == ors2.type

    assert ors1.X == ors2.X
    assert ors1.com == ors2.com
    assert ors1.force == ors2.force
    assert ors1.torque == ors2.torque

    for i, _ in enumerate(ors1.shapes):
        assert_ors_shape_equal(ors1.shapes[i], ors2.shapes[i])


def assert_joint_equal(ros, ors):
    assert type(ros) == ors_msgs.msg.Joint
    assert type(ors) == Joint

    assert ros.index == ors.index
    assert ros.qIndex == ors.qIndex
    assert ros.index_from == ors.ifrom
    assert ros.index_to == ors.ito

    assert ros.agent == ors.agent
    assert ros.name == ors.name
    assert ros.joint_type == ors.type

    assert_transform_equal(ros.A, ros.Avel, ors.A)
    assert_transform_equal(ros.Q, ros.Qvel, ors.Q)
    assert_transform_equal(ros.B, ros.Bvel, ors.B)
    assert_transform_equal(ros.X, ros.Xvel, ors.X)


def assert_ors_joint_equal(ors1, ors2):
    assert ors1.index == ors2.index
    assert ors1.qIndex == ors2.qIndex
    assert ors1.ifrom == ors2.ifrom
    assert ors1.ito == ors2.ito

    assert ors1.agent == ors2.agent
    assert ors1.name == ors2.name
    assert ors1.type == ors2.type

    assert ors1.A == ors2.A
    assert ors1.Q == ors2.Q
    assert ors1.B == ors2.B
    assert ors1.X == ors2.X


def assert_graph_equal(ros, ors):
    for i, _ in enumerate(ros.bodies):
        assert_body_equal(ros.bodies[i], ors.bodies[i])
        assert ors.bodies[i].index == i
        assert ros.bodies[i].index == i

    for i, _ in enumerate(ros.shapes):
        assert_shape_equal(ros.shapes[i], ors.shapes[i])
        assert ors.shapes[i].index == i
        assert ros.shapes[i].index == i

    for i, _ in enumerate(ros.joints):
        assert_joint_equal(ros.joints[i], ors.joints[i])
        assert ors.joints[i].index == i
        assert ros.joints[i].index == i

    assert ros.q_dim == ors.q_dim
    assert ros.isLinkTree == ors.isLinkTree


def assert_ors_graph_equal(ors1, ors2):
    assert len(ors1.joints) == len(ors2.joints)
    for i, _ in enumerate(ors1.bodies):
        assert_ors_body_equal(ors1.bodies[i], ors2.bodies[i])

    assert len(ors1.shapes) == len(ors2.shapes)
    for i, _ in enumerate(ors1.shapes):
        assert_ors_shape_equal(ors1.shapes[i],
                               ors2.shapes[i],
                               check_bodies_set=True)

    assert len(ors1.joints) == len(ors2.joints)
    for i, _ in enumerate(ors1.joints):
        assert_ors_joint_equal(ors1.joints[i], ors2.joints[i])

    assert ors1.q_dim == ors2.q_dim
    assert ors1.isLinkTree == ors2.isLinkTree


###############################################################################
# Real tests
class Test_Vector(unittest.TestCase):
    def test_ors_to_ros(self):
        ors_v1 = Vector(1, 2, 3)
        copy_v = ors_v1

        ros_v1 = parser.ors_to_ros_vector(ors_v1)

        # assert no changes to ors_v1
        assert copy_v == ors_v1

        # assert ros and ors vector values are equal
        assert_vector_equal(ros_v1, ors_v1)

    def test_ros_to_ors(self):
        ros_v1 = geometry_msgs.msg.Vector3()
        ros_v1.x = 1
        ros_v1.y = 2
        ros_v1.z = 3
        copy_v = ros_v1

        ors_v1 = parser.ros_to_ors_vector(ros_v1)

        # assert no changes to ros_v1
        assert copy_v == ros_v1

        # assert ros and ors vector values are equal
        assert_vector_equal(ros_v1, ors_v1)

    def test_ors_ros_ors(self):
        ors_v1 = Vector(1, 2, 3)
        ros_v1 = parser.ors_to_ros_vector(ors_v1)
        ors_v2 = parser.ros_to_ors_vector(ros_v1)

        assert ors_v1 == ors_v2

    def test_ros_ors_ros(self):
        ros_v1 = geometry_msgs.msg.Vector3()
        ros_v1.x = 1
        ros_v1.y = 2
        ros_v1.z = 3

        ors_v1 = parser.ros_to_ors_vector(ros_v1)
        ros_v2 = parser.ors_to_ros_vector(ors_v1)

        assert ros_v1 == ros_v2


class Test_Quaternion(unittest.TestCase):
    def test_ors_to_ros(self):
        ors_q1 = Quaternion(1, 2, 3, 4)
        copy_q = ors_q1

        ros_q1 = parser.ors_to_ros_quaternion(ors_q1)

        # assert no changes to ors_q1
        assert copy_q == ors_q1

        # assert ros and ors vector values are equal
        assert_quaternion_equal(ros_q1, ors_q1)

    def test_ros_to_ors(self):
        ros_q1 = geometry_msgs.msg.Quaternion()
        ros_q1.w = 1
        ros_q1.x = 2
        ros_q1.y = 3
        ros_q1.z = 4

        copy_q = ros_q1

        ors_q1 = parser.ros_to_ors_quaternion(ros_q1)

        # assert no changes to ros_q1
        assert copy_q == ros_q1

        # assert ros and ors vector values are equal
        assert_quaternion_equal(ros_q1, ors_q1)

    def test_ors_ros_ors(self):
        ors_q1 = Quaternion(1, 2, 3, 4)
        ros_q1 = parser.ors_to_ros_quaternion(ors_q1)
        ors_q2 = parser.ros_to_ors_quaternion(ros_q1)

        assert ors_q1 == ors_q2

    def test_ros_ors_ros(self):
        ros_q1 = geometry_msgs.msg.Quaternion()
        ros_q1.w = 1
        ros_q1.x = 2
        ros_q1.y = 3
        ros_q1.z = 4

        ors_q1 = parser.ros_to_ors_quaternion(ros_q1)
        ros_q2 = parser.ors_to_ros_quaternion(ors_q1)

        assert ros_q1 == ros_q2


class Test_Transform(unittest.TestCase):
    def test_ors_to_ros(self):
        ors_rot = Quaternion(1, 2, 3, 4)
        ors_pos = Vector(5, 6, 7)
        ors_vel = Vector(8, 7, 6)
        ors_angvel = Vector(5, 4, 3)

        ors_transform = Transformation()
        ors_transform.pos = ors_pos
        ors_transform.rot = ors_rot
        ors_transform.vel = ors_vel
        ors_transform.angvel = ors_angvel

        copy_transform = ors_transform

        ros_transform, ros_twist = parser.ors_to_ros_transform(ors_transform)

        # assert ors transformation didn't change
        assert copy_transform == ors_transform

        # assert ros msg values are equal to ors_transform
        assert_transform_equal(ros_transform, ros_twist, ors_transform)

    def test_ros_to_ors(self):
        ros_transform = geometry_msgs.msg.Transform()

        ros_transform.rotation.w = 1
        ros_transform.rotation.x = 2
        ros_transform.rotation.y = 3
        ros_transform.rotation.z = 4

        ros_transform.translation.x = 5
        ros_transform.translation.y = 6
        ros_transform.translation.z = 7

        ros_twist = geometry_msgs.msg.Twist()
        ros_twist.linear.x = 8
        ros_twist.linear.y = 9
        ros_twist.linear.z = 1

        ros_twist.angular.x = 2
        ros_twist.angular.y = 3
        ros_twist.angular.z = 4

        copy_transform = ros_transform
        copy_twist = ros_twist

        ors_transform = parser.ros_to_ors_transform(ros_transform, ros_twist)

        # assert ros transformation didn't change
        assert copy_transform == ros_transform
        assert copy_twist == ros_twist

        # assert ros msg values are equal to ors_transform
        assert_transform_equal(ros_transform, ros_twist, ors_transform)

    def test_ros_ors_ros(self):
        ros_transform = geometry_msgs.msg.Transform()

        ros_transform.rotation.w = 1
        ros_transform.rotation.x = 2
        ros_transform.rotation.y = 3
        ros_transform.rotation.z = 4

        ros_transform.translation.x = 5
        ros_transform.translation.y = 6
        ros_transform.translation.z = 7

        ros_twist = geometry_msgs.msg.Twist()
        ros_twist.linear.x = 8
        ros_twist.linear.y = 9
        ros_twist.linear.z = 1

        ros_twist.angular.x = 2
        ros_twist.angular.y = 3
        ros_twist.angular.z = 4

        ors_transform = parser.ros_to_ors_transform(ros_transform, ros_twist)
        ros_transform2, ros_twist2 = parser.ors_to_ros_transform(ors_transform)

        assert ros_transform == ros_transform2
        assert ros_twist == ros_twist2

    def test_ors_ros_ors(self):
        ors_rot = Quaternion(1, 2, 3, 4)
        ors_pos = Vector(5, 6, 7)

        ors_transform = Transformation()
        ors_transform.pos = ors_pos
        ors_transform.rot = ors_rot

        ros_transform, ros_twist = parser.ors_to_ros_transform(ors_transform)
        ors_transform2 = parser.ros_to_ors_transform(ros_transform, ros_twist)

        assert ors_transform == ors_transform2


class Test_PrimitiveShape(unittest.TestCase):
    def test_ors_to_ros(self):
        ors_s1 = Shape()

        ors_s1.index = 1
        ors_s1.ibody = 2
        ors_s1.body = None
        ors_s1.name = "testshape"

        ors_s1.X.pos = Vector(3, 4, 5)
        ors_s1.rel.pos = Vector(6, 7, 8)
        ors_s1.X.rot = Quaternion(1, 9, 1, 2)
        ors_s1.rel.rot = Quaternion(1, 3, 4, 5)

        ors_s1.type = sphereST
        ors_s1.cont = True

        copy_shape = ors_s1

        ros_s1 = parser.ors_shape_to_msg(ors_s1)

        # assert ors_s1 didn't change
        assert copy_shape == ors_s1

        assert_shape_equal(ros_s1, ors_s1)

    def test_ros_to_ors(self):
        shape_msg = ors_msgs.msg.Shape()

        shape_msg.index = 1
        shape_msg.index_body = 2
        shape_msg.name = "testshape"

        shape_msg.X.translation = parser.ors_to_ros_vector(Vector(3, 4, 5))
        shape_msg.X.rotation = parser.ors_to_ros_quaternion(
            Quaternion(1, 6, 7, 8))
        shape_msg.rel.translation = parser.ors_to_ros_vector(
            Vector(9, 1, 2))
        shape_msg.rel.rotation = parser.ors_to_ros_quaternion(
            Quaternion(1, 3, 4, 5))

        shape_msg.shape_type = sphereST
        shape_msg.contact = True

        copy_shape = shape_msg

        ors_shape = parser.msg_to_ors_shape(shape_msg)

        assert copy_shape == shape_msg
        assert_shape_equal(shape_msg, ors_shape)

    def test_ors_ros_ors(self):
        ors_s1 = Shape()

        ors_s1.index = 1
        ors_s1.ibody = 2
        ors_s1.body = None
        ors_s1.name = "testshape"

        ors_s1.X.pos = Vector(3, 4, 5)
        ors_s1.rel.pos = Vector(6, 7, 8)
        ors_s1.X.rot = Quaternion(1, 9, 1, 2)
        ors_s1.rel.rot = Quaternion(1, 3, 4, 5)

        ors_s1.type = sphereST
        ors_s1.cont = True

        copy_shape = ors_s1

        shape_msg = parser.ors_shape_to_msg(ors_s1)
        ors_s2 = parser.msg_to_ors_shape(shape_msg)

        assert copy_shape == ors_s1
        assert_ors_shape_equal(ors_s1, ors_s2)

    def test_ros_ors_ros(self):
        shape_msg = ors_msgs.msg.Shape()

        shape_msg.index = 1
        shape_msg.index_body = 2
        shape_msg.name = "testshape"

        shape_msg.X.translation = parser.ors_to_ros_vector(Vector(3, 4, 5))
        shape_msg.X.rotation = parser.ors_to_ros_quaternion(
            Quaternion(1, 6, 7, 8))
        shape_msg.rel.translation = parser.ors_to_ros_vector(
            Vector(9, 1, 2))
        shape_msg.rel.rotation = parser.ors_to_ros_quaternion(
            Quaternion(1, 3, 4, 5))

        shape_msg.shape_type = sphereST
        shape_msg.contact = True

        copy_shape = shape_msg

        ors_shape = parser.msg_to_ors_shape(shape_msg)
        shape_msg2 = parser.ors_shape_to_msg(ors_shape)

        assert copy_shape == shape_msg
        assert shape_msg == shape_msg2


class Test_Mesh(unittest.TestCase):
    def test_ors_to_ros(self):
        g = Graph("handle.ors")
        ors_mesh = g.shapes[0].mesh

        ros_mesh = parser.ors_mesh_to_msg(ors_mesh)

        assert_mesh_equal(ros_mesh, ors_mesh)

    def test_ors_ros_ors(self):
        g = Graph('handle.ors')
        ors_mesh = g.shapes[0].mesh

        ros_mesh = parser.ors_mesh_to_msg(ors_mesh)
        assert_mesh_equal(ros_mesh, ors_mesh)

        ors_mesh2 = parser.msg_to_ors_mesh(ros_mesh)
        assert_mesh_equal(ros_mesh, ors_mesh2)


class Test_MeshShape(unittest.TestCase):
    def test_ors_to_ros(self):
        g = Graph('handle.ors')
        ors_s1 = g.shapes[0]

        copy_shape = ors_s1

        ros_s1 = parser.ors_shape_to_msg(ors_s1)

        assert_ors_shape_equal(copy_shape, ors_s1)
        assert_shape_equal(ros_s1, ors_s1)

    def test_ors_ros_ors(self):
        g = Graph('handle.ors')
        ors_s1 = g.shapes[0]

        shape_msg = parser.ors_shape_to_msg(ors_s1)
        assert_shape_equal(shape_msg, ors_s1)

        ors_s2 = parser.msg_to_ors_shape(shape_msg)
        assert_shape_equal(shape_msg, ors_s2)

        assert_ors_shape_equal(ors_s1, ors_s2)


class Test_Body(unittest.TestCase):
    def test_ors_to_ros(self):
        g = Graph('handle.ors')
        ors_b1 = g.bodies[0]

        copy_body = ors_b1

        body_msg = parser.ors_body_to_msg(ors_b1)

        assert ors_b1 == copy_body
        assert_body_equal(body_msg, ors_b1)

    def test_ors_ros_ors(self):
        g = Graph('handle.ors')
        ors_b1 = g.bodies[0]

        copy_body = ors_b1

        body_msg = parser.ors_body_to_msg(ors_b1)
        ors_b2 = parser.msg_to_ors_body(body_msg)

        assert_ors_body_equal(ors_b1, copy_body)
        assert_body_equal(body_msg, ors_b1)
        assert_body_equal(body_msg, ors_b2)
        assert_ors_body_equal(ors_b1, ors_b2)


class Test_Joint(unittest.TestCase):
    def test_ors_to_ros(self):
        g = Graph('handle.ors')
        ors_j1 = g.joints[0]

        copy_joint = ors_j1

        joint_msg = parser.ors_joint_to_msg(ors_j1)

        assert ors_j1 == copy_joint
        assert_joint_equal(joint_msg, ors_j1)


class Test_Graph(unittest.TestCase):
    def test_ors_to_ros(self):
        g = Graph("handle.ors")
        copy = g

        msg = parser.ors_graph_to_msg(g)

        assert g == copy
        assert_graph_equal(msg, g)

    def test_ors_ros_ors(self):
        g1 = Graph("handle.ors")
        copy = g1

        msg = parser.ors_graph_to_msg(g1)
        g2 = parser.msg_to_ors_graph(msg)

        assert_ors_graph_equal(g1, g2)
        assert g1 == copy

        for joint in g2.joints:
            assert_ors_body_equal(g2.bodies[joint.ifrom], joint._from)
            assert_ors_body_equal(g2.bodies[joint.ito], joint.to)

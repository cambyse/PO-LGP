import unittest

import roslib
roslib.load_manifest("rosors")

from orspy import Shape, sphereST, meshST
from corepy import Vector, Quaternion, Transformation
from rosors import parser

import geometry_msgs.msg
import ors_msgs.msg


def assert_vector_equal(ros, ors):
    assert ros.x == ors.x
    assert ros.y == ors.y
    assert ros.z == ors.z


def assert_quaternion_equal(ros, ors):
    assert ros.w == ors.w
    assert ros.x == ors.x
    assert ros.y == ors.y
    assert ros.z == ors.z


def assert_transform_equal(ros, ors):
    assert_vector_equal(ros.translation, ors.pos)
    assert_quaternion_equal(ros.rotation, ors.rot)


def assert_mesh_equal(ros, ors):
    pass


def assert_shape_equal(ros, ors):
    assert ros.index == ors.index
    assert ros.index_body == ors.ibody
    assert ros.name == ors.name
    assert_transform_equal(ros.X, ors.X)
    assert_transform_equal(ros.rel, ors.rel)
    assert ros.shape_type == ors.type
    assert ros.contact == ors.cont
    if ors.type == meshST or ros.shape_type == meshST:
        assert_mesh_equal(ros.mesh, ors.mesh)


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

        ors_transform = Transformation()
        ors_transform.pos = ors_pos
        ors_transform.rot = ors_rot

        copy_transform = ors_transform

        ros_transform = parser.ors_to_ros_transform(ors_transform)

        # assert ors transformation didn't change
        assert copy_transform == ors_transform

        # assert ros msg values are equal to ors_transform
        assert_transform_equal(ros_transform, ors_transform)

    def test_ros_to_ors(self):
        ros_transform = geometry_msgs.msg.Transform()

        ros_transform.rotation.w = 1
        ros_transform.rotation.x = 2
        ros_transform.rotation.y = 3
        ros_transform.rotation.z = 4

        ros_transform.translation.x = 5
        ros_transform.translation.y = 6
        ros_transform.translation.z = 7

        copy_transform = ros_transform

        ors_transform = parser.ros_to_ors_transform(ros_transform)

        # assert ros transformation didn't change
        assert copy_transform == ros_transform

        # assert ros msg values are equal to ors_transform
        assert_transform_equal(ros_transform, ors_transform)

    def test_ros_ors_ros(self):
        ros_transform = geometry_msgs.msg.Transform()

        ros_transform.rotation.w = 1
        ros_transform.rotation.x = 2
        ros_transform.rotation.y = 3
        ros_transform.rotation.z = 4

        ros_transform.translation.x = 5
        ros_transform.translation.y = 6
        ros_transform.translation.z = 7

        ors_transform = parser.ros_to_ors_transform(ros_transform)
        ros_transform2 = parser.ors_to_ros_transform(ors_transform)

        assert ros_transform == ros_transform2

    def test_ors_ros_ors(self):
        ors_rot = Quaternion(1, 2, 3, 4)
        ors_pos = Vector(5, 6, 7)

        ors_transform = Transformation()
        ors_transform.pos = ors_pos
        ors_transform.rot = ors_rot

        ros_transform = parser.ors_to_ros_transform(ors_transform)
        ors_transform2 = parser.ros_to_ors_transform(ros_transform)

        assert ors_transform == ors_transform2


class Test_PrimitiveShape(unittest.TestCase):
    def test_ors_to_ros(self):
        ors_s1 = Shape()

        ors_s1.index = 1
        ors_s1.ibody = 2
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
        assert ors_s1 == ors_s2


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


if __name__ == '__main__':
    unittest.main()

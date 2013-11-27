"""
Helpers for creating ros msgs and ors datastructures.

You can use the helper functions from whereever you want.
"""

# ROS
import roslib
roslib.load_manifest("rosors")
import geometry_msgs.msg
import shape_msgs
# MLR
import ors_msgs.msg
import orspy
from corepy import Transformation, Vector, Quaternion
from guipy import Mesh
import numpy as np


def ros_to_ors_vector(ros_vector):
    return Vector(ros_vector.x,
                  ros_vector.y,
                  ros_vector.z)


def ors_to_ros_vector(ors_vector):
    ros_vector = geometry_msgs.msg.Vector3()
    ros_vector.x = ors_vector.x
    ros_vector.y = ors_vector.y
    ros_vector.z = ors_vector.z
    return ros_vector


def ros_to_ors_quaternion(ros_quaternion):
    return Quaternion(ros_quaternion.w,
                      ros_quaternion.x,
                      ros_quaternion.y,
                      ros_quaternion.z)


def ors_to_ros_quaternion(ors_quaternion):
    ros_quaternion = geometry_msgs.msg.Quaternion()
    ros_quaternion.w = ors_quaternion.w
    ros_quaternion.x = ors_quaternion.x
    ros_quaternion.y = ors_quaternion.y
    ros_quaternion.z = ors_quaternion.z
    return ros_quaternion


def ros_to_ors_transform(ros_transform, ros_twist):
    ors_transform = Transformation()
    ors_transform.pos = ros_to_ors_vector(ros_transform.translation)
    ors_transform.rot = ros_to_ors_quaternion(ros_transform.rotation)
    ors_transform.vel = ros_to_ors_vector(ros_twist.linear)
    ors_transform.angvel = ros_to_ors_vector(ros_twist.angular)
    return ors_transform


def ors_to_ros_transform(ors_transform):
    ros_transform = geometry_msgs.msg.Transform()
    ros_transform.translation = ors_to_ros_vector(ors_transform.pos)
    ros_transform.rotation = ors_to_ros_quaternion(ors_transform.rot)
    ros_twist = geometry_msgs.msg.Twist()
    ros_twist.linear = ors_to_ros_vector(ors_transform.vel)
    ros_twist.angular = ors_to_ros_vector(ors_transform.angvel)
    return ros_transform, ros_twist


def ors_mesh_to_msg(ors_mesh):
    mesh_msg = shape_msgs.msg.Mesh()
    # vertices
    for i in range(ors_mesh.V.shape[0]):
        vertex = geometry_msgs.msg.Point(
            ors_mesh.V[i, 0],
            ors_mesh.V[i, 1],
            ors_mesh.V[i, 2]
        )
        mesh_msg.vertices.append(vertex)
    # triangles/faces
    for i in range(ors_mesh.T.shape[0]):
        triangle = shape_msgs.msg.MeshTriangle([
            ors_mesh.T[i, 0],
            ors_mesh.T[i, 1],
            ors_mesh.T[i, 2]
        ])
        mesh_msg.triangles.append(triangle)
    return mesh_msg


def msg_to_ors_mesh(mesh_msg):
    mesh = Mesh()
    V = np.resize(mesh.V, [len(mesh_msg.vertices), 3])
    T = np.resize(mesh.T, [len(mesh_msg.triangles), 3])

    for i in range(len(mesh_msg.vertices)):
        v = mesh_msg.vertices[i]
        V[i, 0] = v.x
        V[i, 1] = v.y
        V[i, 2] = v.z
    mesh.V = V  # need to assign members, because of swig

    for i in range(len(mesh_msg.triangles)):
        t = mesh_msg.triangles[i]
        T[i, 0] = t.vertex_indices[0]
        T[i, 1] = t.vertex_indices[1]
        T[i, 2] = t.vertex_indices[2]
    mesh.T = T  # see above
    return mesh


def ors_shape_to_msg(ors_shape):
    shape_msg = ors_msgs.msg.Shape()

    shape_msg.index = ors_shape.index
    shape_msg.index_body = ors_shape.ibody
    shape_msg.name = ors_shape.name

    shape_msg.X, shape_msg.Xvel = ors_to_ros_transform(ors_shape.X)
    shape_msg.rel, shape_msg.relvel = ors_to_ros_transform(ors_shape.rel)

    shape_msg.shape_type = ors_shape.type
    shape_msg.contact = ors_shape.cont

    if ors_shape.type == orspy.meshST:
        shape_msg.mesh = ors_mesh_to_msg(ors_shape.mesh)

    return shape_msg


def msg_to_ors_shape(msg, graph=None, body=None):
    shape = orspy.Shape()
    shape.index = msg.index
    shape.ibody = msg.index_body
    shape.name = msg.name

    shape.body = None

    shape.X = ros_to_ors_transform(msg.X, msg.Xvel)
    shape.rel = ros_to_ors_transform(msg.rel, msg.relvel)

    shape.type = msg.shape_type
    shape.cont = msg.contact
    if shape.type == orspy.meshST:
        shape.mesh = msg_to_ors_mesh(msg.mesh)
        shape.mesh.thisown = False

    return shape


def ors_body_to_msg(ors_body):
    body_msg = ors_msgs.msg.Body()

    body_msg.index = ors_body.index
    body_msg.name = ors_body.name
    body_msg.mass = ors_body.mass
    body_msg.body_type = ors_body.type

    # ors transformation
    body_msg.transform, body_msg.twist = ors_to_ros_transform(ors_body.X)

    body_msg.com = ors_to_ros_vector(ors_body.com)
    body_msg.force = ors_to_ros_vector(ors_body.force)
    body_msg.torque = ors_to_ros_vector(ors_body.torque)

    # shapes
    for shape in ors_body.shapes:
        body_msg.shapes.append(ors_shape_to_msg(shape))

    return body_msg


def msg_to_ors_body(msg, graph=None):
    ors_body = orspy.Body()

    ors_body.index = msg.index
    ors_body.name = msg.name
    ors_body.mass = msg.mass
    ors_body.type = msg.body_type

    ors_body.X = ros_to_ors_transform(msg.transform, msg.twist)

    ors_body.com = ros_to_ors_vector(msg.com)
    ors_body.force = ros_to_ors_vector(msg.force)
    ors_body.torque = ros_to_ors_vector(msg.torque)

    shapes = []
    for shape in msg.shapes:
        shapes.append(msg_to_ors_shape(shape))
        shapes[-1].thisown = False
    ors_body.shapes = shapes

    return ors_body


def msg_to_ors_joint(msg):
    ors_joint = orspy.Joint()

    ors_joint.index = msg.index
    ors_joint.qIndex = msg.qIndex
    ors_joint.ifrom = msg.index_from
    ors_joint.ito = msg.index_to

    ors_joint.agent = msg.agent
    ors_joint.name = msg.name

    ors_joint.type = msg.joint_type

    ors_joint.A = ros_to_ors_transform(msg.A, msg.Avel)
    ors_joint.Q = ros_to_ors_transform(msg.Q, msg.Qvel)
    ors_joint.B = ros_to_ors_transform(msg.B, msg.Bvel)
    ors_joint.X = ros_to_ors_transform(msg.X, msg.Xvel)

    ors_joint.axis = ros_to_ors_vector(msg.axis)

    ors_joint.to = None
    ors_joint._from = None

    return ors_joint


def ors_joint_to_msg(ors_joint):
    joint_msg = ors_msgs.msg.Joint()

    joint_msg.index = ors_joint.index
    joint_msg.qIndex = ors_joint.qIndex
    joint_msg.index_from = ors_joint.ifrom
    joint_msg.index_to = ors_joint.ito

    joint_msg.agent = ors_joint.agent
    joint_msg.name = ors_joint.name
    joint_msg.joint_type = ors_joint.type

    joint_msg.A, joint_msg.Avel = ors_to_ros_transform(ors_joint.A)
    joint_msg.Q, joint_msg.Qvel = ors_to_ros_transform(ors_joint.Q)
    joint_msg.B, joint_msg.Bvel = ors_to_ros_transform(ors_joint.B)
    joint_msg.X, joint_msg.Xvel = ors_to_ros_transform(ors_joint.X)

    joint_msg.axis = ors_to_ros_vector(ors_joint.axis)

    return joint_msg


def msg_to_ors_graph(msg):
    graph = orspy.Graph()

    bodies = []
    shapes = [None]*len(msg.shapes)
    for body in msg.bodies:
        bodies.append(msg_to_ors_body(body))
        bodies[-1].thisown = False
        for shape in bodies[-1].shapes:
            shape.body = bodies[-1]
            shapes[shape.index] = shape

    graph.bodies = bodies
    graph.shapes = shapes

    joints = []
    for joint in msg.joints:
        joints.append(msg_to_ors_joint(joint))
        joints[-1].thisown = False
    graph.joints = joints

    # post-process after assignment
    for joint in graph.joints:
        joint._from = graph.bodies[joint.ifrom]
        joint.to = graph.bodies[joint.ito]
        graph.bodies[joint.ifrom].outLinks += [joint]  # don't use append(),
                                                       # because of annoying
                                                       # swig typemapped
                                                       # members
        graph.bodies[joint.ito].inLinks += [joint]

    for shape in graph.shapes:
        shape.body = graph.bodies[shape.ibody]

    graph.q_dim = msg.q_dim
    graph.isLinkTree = msg.isLinkTree

    return graph


def ors_graph_to_msg(graph):
    msg = ors_msgs.msg.Graph()

    for body in graph.bodies:
        msg.bodies.append(ors_body_to_msg(body))

    for shape in graph.shapes:
        msg.shapes.append(ors_shape_to_msg(shape))

    for joint in graph.joints:
        msg.joints.append(ors_joint_to_msg(joint))

    msg.q_dim = graph.q_dim
    msg.isLinkTree = graph.isLinkTree

    return msg

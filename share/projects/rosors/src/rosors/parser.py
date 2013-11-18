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


def ros_to_ors_transform(ros_transform):
    ors_transform = Transformation()
    ors_transform.pos = ros_to_ors_vector(ros_transform.translation)
    ors_transform.rot = ros_to_ors_quaternion(ros_transform.rotation)
    return ors_transform


def ors_to_ros_transform(ors_transform):
    ros_transform = geometry_msgs.msg.Transform()
    ros_transform.translation = ors_to_ros_vector(ors_transform.pos)
    ros_transform.rotation = ors_to_ros_quaternion(ors_transform.rot)
    return ros_transform


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
    mesh = orspy.Mesh()
    mesh.V = mesh.V.reshape([3, len(mesh_msg.vertices)])
    mesh.T = mesh.T.reshape([3, len(mesh_msg.triangles)])
    for i in range(len(mesh_msg.vertices)):
        v = mesh_msg.vertices[i]
        mesh.V[i, 0] = v.x
        mesh.V[i, 1] = v.y
        mesh.V[i, 2] = v.z
    for i in range(len(mesh_msg.triangles)):
        t = mesh_msg.triangles[i]
        mesh.T[i, 0] = t.vertex_indices[0]
        mesh.T[i, 1] = t.vertex_indices[1]
        mesh.T[i, 2] = t.vertex_indices[2]
    return mesh


def ors_shape_to_msg(ors_shape):
    shape_msg = ors_msgs.msg.Shape()

    shape_msg.index = ors_shape.index
    shape_msg.index_body = ors_shape.ibody
    shape_msg.name = ors_shape.name

    shape_msg.X = ors_to_ros_transform(ors_shape.X)
    shape_msg.rel = ors_to_ros_transform(ors_shape.rel)

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

    shape.body = body
    if graph is not None:
        #TODO: resize!
        graph.shapes[msg.index] = shape
        shape.body = graph.bodies[msg.index_body]

    shape.X = ros_to_ors_transform(msg.X)
    shape.rel = ros_to_ors_transform(msg.rel)

    shape.type = msg.shape_type
    shape.cont = msg.contact
    if shape.type == orspy.meshST:
        shape.mesh = msg_to_ors_mesh(msg.mesh)

    return shape


def ors_body_to_msg(ors_body):
    body_msg = ors_msgs.msg.Body()

    body_msg.index = ors_body.index
    body_msg.name = ors_body.name
    body_msg.mass = ors_body.mass
    body_msg.body_type = ors_body.type

    # ors transformation
    body_msg.pos = ors_to_ros_vector(ors_body.X.pos)
    body_msg.rot = ors_to_ros_quaternion(ors_body.X.rot)
    body_msg.vel = ors_to_ros_vector(ors_body.X.vel)
    body_msg.angvel = ors_to_ros_vector(ors_body.X.angvel)

    body_msg.com = ors_body.com

    body_msg.force = ors_body.force
    body_msg.torque = ors_body.torque

    # shapes
    for shape in ors_body.shapes:
        body_msg.shapes.append(ors_shape_to_msg(shape))

    return body_msg


def msg_to_ors_body(msg, graph=None):
    ors_body = orspy.Body()

    ors_body.index = msg.index
    if graph is not None:
        #TODO: resize!
        graph.bodies[ors_body.index] = ors_body

    ors_body.name = msg.name
    ors_body.mass = msg.mass
    ors_body.type = msg.body_type

    return ors_body


def msg_to_ors_graph(msg):
    graph = orspy.Graph()
    for body in msg.bodies:
        graph.bodies.append(msg_to_ors_body(body, graph))

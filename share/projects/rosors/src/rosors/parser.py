"""
Helpers for creating ros msgs and ors datastructures.

You can use the helper functions from whereever you want.
"""

# ROS
import roslib
roslib.load_manifest("rosors")
import geometry_msgs
import shape_msgs
# MLR
import ors_msgs
import orspy


def ors_shape_to_msg(ors_shape):
    shape_msg = ors_msgs.msg.Shape()

    shape_msg.index = ors_shape.index
    shape_msg.index_body = ors_shape.ibody
    shape_msg.name = ors_shape.name

    shape_msg.X.translation = ors_shape.X.pos
    shape_msg.X.rotation = ors_shape.X.rot
    shape_msg.rel.translation = ors_shape.rel.pos
    shape_msg.rel.rotation = ors_shape.rel.rot

    shape_msg.shape_type = ors_shape.type
    if ors_shape.type == orspy.meshST:
        # vertices
        for i in range(ors_shape.mesh.V.shape[0]):
            vertex = geometry_msgs.msg.Point(
                ors_shape.mesh.V[i, 0],
                ors_shape.mesh.V[i, 1],
                ors_shape.mesh.V[i, 2]
            )
            shape_msg.mesh.vertices.append(vertex)
        # triangles/faces
        for i in range(ors_shape.mesh.T.shape[0]):
            triangle = shape_msgs.msg.MeshTriangle([
                ors_shape.mesh.T[i, 0],
                ors_shape.mesh.T[i, 1],
                ors_shape.mesh.T[i, 2]
            ])
            shape_msg.mesh.triangles.append(triangle)

    return shape_msg


def ors_body_to_msg(ors_body):
    body_msg = ors_msgs.msg.Body()

    body_msg.index = ors_body.index
    body_msg.name = ors_body.name
    body_msg.mass = ors_body.mass

    # ors transformation
    body_msg.pos = ors_body.X.pos
    body_msg.rot = ors_body.X.rot
    body_msg.vel = ors_body.X.vel
    body_msg.angvel = ors_body.X.angvel

    body_msg.com = ors_body.com

    body_msg.force = ors_body.force
    body_msg.torque = ors_body.torque

    # shapes
    for shape in ors_body.shapes:
        body_msg.shapes.append(ors_shape_to_msg(shape))

    return body_msg

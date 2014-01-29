import orspy as ors
import corepy
import rospy
import the_curious_robot.msg as msgs
import geometry_msgs.msg
from articulation_msgs.msg import TrackMsg
import numpy as np


class Properties(object):
    """A collection of potential properties a DoF can have."""
    def __init__(self):
        self.joint = None  # ss.norm(loc=?, scale=)
        self.friction = None  # ss.norm(loc=?, scale=)
        self.weight = None  # ss.norm(loc=?, scale=)
        self.limit_min = None  # ss.norm(loc=?, scale=)
        self.limit_max = None  # ss.norm(loc=?, scale=)

    def property_names(self):
        return [attr for attr in dir(self)
                if not (attr.startswith("__") or
                        callable(getattr(self, attr)))]


############################################################################
# Note:
# The vanialla ors __str__ spits out the "content" of the ors datatype
# but it does not contain the name (and some other information). Therefore we
# have a bunch of parse_DATATYPE_msg and create_DATATYPE_msg function.
#
# TODO: find something better than str to transport ors datastructures!

############################################################################
# msg parser/builder
def parse_body_msg(body_str):
    """
    Transform the string representation into an ors.Body.
    """
    body = ors.Body()
    name, properties = body_str.split(' ', 1)
    body.name = name
    body.read(properties)
    return body


def create_body_msg(body):
    """
    Transform the ors body into a sensible string representation.

    The vanialla ors __str__ spits out the "content" of the ors datatype but
    does not contain the name.
    """
    return body.name + ' ' + str(body)


#########################################################################
def parse_shape_msg(all_shapes_msg):
    result = []
    for shape_str in all_shapes_msg.shapes:
        name, idx, properties = shape_str.split(" ", 2)
        shape = ors.Shape()
        shape.read(properties)
        result.append(shape)
    return result


def create_shape_msg(shape):
    return "{} {} {}".format(shape.body.name, str(shape.index), str(shape))


#########################################################################
def parse_property_msg(msg):
    properties = Properties()
    for p in msg:
        setattr(properties, p.name, ss.norm())  # p.values[0], p.values[1]))
    return properties


def create_properties_msg(properties):
    result = []
    for prop_name in properties.property_names():
        msg = msgs.Property()
        msg.name = prop_name
        # msg.values = [getattr(properties, prop_name).mu,
        #               getattr(properties, prop_name).sigma]
        result.append(msg)
    return result


#########################################################################
def parse_ooi_msg(msg):
    body = parse_body_msg(msg.body)
    properties = parse_property_msg(msg.properties)
    return {"body": body, "properties": properties}


def create_ooi_msg(body, properties):
    body_msg = create_body_msg(body)
    properties_msg = create_properties_msg(properties)
    msg = msgs.Object()
    msg.body = body_msg
    msg.properties = properties_msg
    return msg


#########################################################################
def parse_oois_msg(msg):
    objects = []
    for obj in msg.objects:
        objects.append(parse_ooi_msg(obj))
    return objects


def create_oois_msg(objects):
    msg = msgs.Objects()
    for obj in objects:
        msg.objects.append(create_ooi_msg(obj['body'], obj['properties']))
    return msg


#########################################################################
def parse_trajectory_msg(msg):
    trajectory = []
    for p in msg.pos:
        pose = corepy.Transformation()
        pose.pos.x = p.position.x
        pose.pos.y = p.position.y
        pose.pos.z = p.position.z
        trajectory.append(pose)
    return msg.object_id, trajectory


def create_trajectory_msg(obj_id, pos):
    msg = msgs.Trajectory()
    for p in pos:
        pose = geometry_msgs.msg.Pose()
        pose.position.x = p.pos.x
        pose.position.y = p.pos.y
        pose.position.z = p.pos.z
        msg.pos.append(pose)
    msg.object_id = obj_id
    return msg


#########################################################################
def create_track_msg(trajectory):
    msg = TrackMsg()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = '/'
    #msg.id = model

    for t in trajectory:
        pose = geometry_msgs.msg.Pose(
            t.pos,
            geometry_msgs.msg.Quaternion(0, 0, 0, 1)
        )
        msg.pose.append(pose)
    return msg


#########################################################################
def shorten_trajectory(traj, num):
    if traj.shape[0] < num:
        return traj
    stepsize = traj.shape[0] / num
    new_traj = np.ndarray([num, traj.shape[1]])
    for i in range(num):
        pos = int(i * stepsize)
        new_traj[i, :] = traj[pos, :]
    return new_traj

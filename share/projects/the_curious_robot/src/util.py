import orspy as ors
import corepy
import rospy
import the_curious_robot.msg as msgs
import geometry_msgs.msg
from articulation_msgs.msg import TrackMsg

import scipy.stats as ss


class ObjectTypeHypo():
    """
    ObjectType represents the probability that an object has a certain type.

    Each object in the world can be either STATIC or FREE. We use a beta
    distribution to model the probability for the object types.
    """
    STATIC = 0
    FREE = 1

    def __init__(self):
        # uninformed prior for beta distribution
        self.alpha = 1
        self.beta = 1
        self.dist = ss.beta(self.alpha, self.beta)

    def update(self, OBJECT_TYPE):
        if OBJECT_TYPE == ObjectTypeHypo.STATIC:
            self.alpha += 1
        elif OBJECT_TYPE == ObjectTypeHypo.FREE:
            self.beta += 1
        else:
            raise TypeError("Type most be STATIC or FREE")

    def __str__(self):
        self.dist = ss.beta(self.alpha, self.beta)
        return "(alpha={}; beta={}; H={})".format(
            self.alpha, self.beta, self.dist.entropy()
        )


class Properties():
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


#########################################################################
# msg parser/builder
def parse_body_msg(msg):
    body = ors.Body()
    body_list = msg.split(' ', 1)
    body.name = body_list[0]
    body.read(body_list[1])
    return body


def create_body_msg(body):
    return body.name + ' ' + str(body)


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

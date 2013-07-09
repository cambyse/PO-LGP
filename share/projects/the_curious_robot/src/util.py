import orspy as ors
import corepy

import roslib
import rospy
import the_curious_robot.msg as msgs
import geometry_msgs.msg
from articulation_msgs.msg import ModelMsg, TrackMsg

class Gaussian():
    """
    Represent Gaussians with this class.
    TODO maybe there is a Gaussian class that we can recycle.
    """
    def __init__(self, mu=0, sigma=99999):
        self.mu = mu
        self.sigma = sigma


class Properties():
    """Potential properties a DoF can have"""
    def __init__(self):
        self.joint = Gaussian()
        self.friction = Gaussian()
        self.weight = Gaussian()
        self.limit_min = Gaussian()
        self.limit_max = Gaussian()

    def property_names(self):
        return [attr for attr in dir(self)
                if not (attr.startswith("__") or
                        callable(getattr(self, attr)))]




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
        setattr(properties, p.name, Gaussian(p.values[0], p.values[1]))
    return properties

def create_properties_msg(properties):
    result = []
    for prop_name in properties.property_names():
        msg = msgs.Property()
        msg.name = prop_name
        msg.values = [ getattr(properties, prop_name).mu,
                       getattr(properties, prop_name).sigma ]
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
    return (msg.object, trajectory)

def create_trajectory_msg(obj_id, pos):
    msg = msgs.Trajectory()
    for p in pos:
        pose = geometry_msgs.msg.Pose()
        pose.position.x = p.pos.x
        pose.position.y = p.pos.y
        pose.position.z = p.pos.z
        msg.pos.append(pose)
    msg.object = obj_id
    return msg

def create_track_msg(trajectory):
    msg = TrackMsg()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = '/'
    #msg.id = model

    for t in trajectory:
        pose = geometry_msgs.msg.Pose(t.pos, geometry_msgs.msg.Quaternion(0, 0,
            0, 1))
        msg.pose.append(pose)
    return msg

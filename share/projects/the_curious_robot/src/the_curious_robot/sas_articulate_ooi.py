#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
from actionlib import SimpleActionServer

from the_curious_robot.msg import ArticulateOOIAction
import require_provide as rp
import rospy
import rosors.srv
import the_curious_robot.msg as msgs

import corepy


class ArticulateOOIActionServer(object):

    def __init__(self, name):
        self.ooi_id = ""
        self.oois = []
        self.control_done = False
        self.react_to_controller = False

        # services
        self.response_shapes = rospy.ServiceProxy("/world/shapes",
                                                  rosors.srv.Shapes)
        self.response_bodies = rospy.ServiceProxy("/world/bodies",
                                                  rosors.srv.Bodies)

        # subscriber
        self.ooi_id_sub = rospy.Subscriber(
            'ooi_id', msgs.ObjectID, self.ooi_id_cb)
        self.control_done_sub = rospy.Subscriber(
            'control_done',
            msgs.control_done, self.control_done_cb)

        # publisher
        self.control_pub = rospy.Publisher('control', msgs.control)
        self.server = SimpleActionServer(
            name, ArticulateOOIAction,
            execute_cb=self.execute, auto_start=False)
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()
        rp.Provide("ArticulateOOI")

        self.endeffector = corepy.getStringParameter("endeffector")

    def execute(self, msg):
        if not self.ooi_id:
            self.server.set_aborted()
            return

        shapes = self.response_shapes(name=self.endeffector)
        start = shapes.shapes[0].X

        shapes = self.response_shapes(index=self.ooi_id)
        self.move_to(shapes.shapes[0].X)
        self.move_to(start)

        self.server.set_succeeded()

    def ooi_id_cb(self, msg):
        self.ooi_id = msg.id

    def control_done_cb(self, msg):
        if self.react_to_controller:
            self.control_done = True

    def preempt_cb(self):
        self.server.set_preempted()

    def move_to(self, target):
        msg = msgs.control()
        msg.pose = target
        msg.ignore_collisions = True
        msg.teleport = False
        msg.endeffector = self.endeffector

        self.react_to_controller = True  # TODO: rather block?
        self.control_pub.publish(msg)

        while not self.control_done and not rospy.is_shutdown():
            rospy.sleep(.1)
        self.react_to_controller = False
        self.control_done = False


def main():
    rospy.init_node('tcr_sas_articulate_ooi')
    ArticulateOOIActionServer('articulate_ooi')

if __name__ == '__main__':
    main()
    rospy.spin()

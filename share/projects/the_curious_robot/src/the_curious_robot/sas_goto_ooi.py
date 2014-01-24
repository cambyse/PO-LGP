#!/usr/bin/env python

import roslib
roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
from actionlib import SimpleActionServer
import the_curious_robot.msg as msgs

import rosors.srv

import require_provide as rp

import corepy


class GotoOOIActionServer:

    def __init__(self, name):
        self.ooi_id = ""
        self.oois = []
        self.control_done = False
        self.react_to_controller = False

        # services
        self.response_shapes = rospy.ServiceProxy("/world/shapes",
                                                  rosors.srv.Shapes)

        # subscriber
        self.ooi_id_sub = rospy.Subscriber(
            'ooi_id', msgs.ObjectID, self.ooi_id_cb)
        self.control_done_sub = rospy.Subscriber(
            'control_done',
            msgs.control_done, self.control_done_cb)

        # publisher
        self.control_pub = rospy.Publisher('control', msgs.control)

        self.server = SimpleActionServer(
            name, msgs.GotoOOIAction,
            execute_cb=self.execute, auto_start=False)
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()
        rp.Provide("GotoOOI")

        self.endeffector = corepy.getStringParameter("endeffector")

    def execute(self, msg):
        if not self.ooi_id:
            self.server.set_aborted()
            return

        shapes = self.response_shapes(index=self.ooi_id)
        msg = msgs.control()
        msg.pose = shapes.shapes[0].X
        msg.ignore_collisions = False
        msg.teleport = corepy.getBoolParameter("teleport", False)
        msg.collision_shapes = corepy.getIntAParameter("agent_shapes")
        msg.endeffector = self.endeffector

        self.react_to_controller = True  # TODO: rather block?
        self.control_pub.publish(msg)

        while not self.control_done and not rospy.is_shutdown():
            rospy.sleep(.1)
        self.react_to_controller = False
        self.control_done = False

        self.server.set_succeeded()

    def ooi_id_cb(self, msg):
        self.ooi_id = msg.id

    def control_done_cb(self, msg):
        if self.react_to_controller:
            self.control_done = True

    def preempt_cb(self):
        self.server.set_preempted()


def main():
    rospy.init_node('tcr_sas_goto_ooi')
    GotoOOIActionServer('goto_ooi')


if __name__ == '__main__':
    main()
    rospy.spin()

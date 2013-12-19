#!/usr/bin/env python
# encoding: utf-8

import roslib

roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
import threading

import rosors.srv
from rosors import parser

from actionlib import SimpleActionServer
import the_curious_robot.msg as msgs
import corepy
import util
import require_provide as rp
from timer import Timer


def get_ooi(oois, ooi_id):
    # TODO what are we doing here and why?
    for ooi in oois:
        if ooi["body"].name == ooi_id:
            return ooi


class ObserveOOITrajActionServer:

    def __init__(self, name):
        # Subscriber
        self.perception_sub = rospy.Subscriber(
            name='/perception/updates',
            data_class=msgs.Objects,
            callback=self.percept_cb)

        self.ooi_id_sub = rospy.Subscriber(
            'ooi_id', msgs.ObjectID, self.ooi_id_cb)

        # Publisher
        self.trajectory_pub = rospy.Publisher(
            'ooi_trajectory', msgs.Trajectory
        )

        # real members
        self.ooi_id = ""
        self.trajectory = []
        self.trajectory_lock = threading.Lock()
        self.data_changed = True

        # ActionLib Server
        self.server = SimpleActionServer(
            name, msgs.ObserveOOITrajAction,
            execute_cb=self.execute, auto_start=False)
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        rp.Provide('ObserveOOITraj')

    def execute(self, msg):
        with Timer("While waiting for end of change", rospy.loginfo):
            while self.data_changed and not rospy.is_shutdown():
                rospy.sleep(.1)

        with Timer("Create and send trajectory", rospy.loginfo):
            with self.trajectory_lock:
                trajectory_msg = util.create_trajectory_msg(
                    self.ooi_id, self.trajectory
                )
                self.trajectory_pub.publish(trajectory_msg)

                # clean up
                del self.trajectory[:]
                self.trajectory = []

        self.server.set_succeeded()

    def percept_cb(self, data):
        self.data_changed = data.changed
        if data.objects:
            for obj in data.objects:
                if obj != self.ooi_id:
                    continue
                with self.trajectory_lock:
                    rospy.logdebug("interesting movement detected")
                    shape_srv = rospy.ServiceProxy("/world/shapes",
                                                   rosors.srv.Shapes)
                    shape_msg = shape_srv(index=self.ooi_id)
                    shape = parser.msg_to_ors_shape(shape_msg.shapes[0])
                    self.trajectory.append(corepy.Transformation(shape.X))
        #rospy.loginfo(self.trajectory)

    def preempt_cb(self):
        self.server.set_preempted()

    def ooi_id_cb(self, msg):
        rospy.logdebug("ooi id = " + str(msg.id))
        self.ooi_id = msg.id


def main():
    rospy.init_node('tcr_sas_observe_ooi_traj')
    ObserveOOITrajActionServer('observe_ooi_traj')

if __name__ == '__main__':
    main()
    rospy.spin()

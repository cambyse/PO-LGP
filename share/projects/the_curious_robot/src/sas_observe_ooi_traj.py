#!/usr/bin/env python

import roslib

roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
import threading

from actionlib import SimpleActionServer
import the_curious_robot.msg as msgs
import orspy
import corepy
import util
import require_provide as rp


def get_ooi(oois, ooi_id):
    for ooi in oois:
        if ooi["body"].name == ooi_id:
            return ooi


class ObserveOOITrajActionServer:

    def __init__(self, name):
        # Subscriber
        self.perception_sub = rospy.Subscriber(
            name='perception_updates',
            data_class=msgs.percept,
            callback=self.percept_cb)
        self.ooi_id_sub = rospy.Subscriber(
            'ooi_id', msgs.ObjectID, self.ooi_id_cb)
        self.oois_sub = rospy.Subscriber(
            'oois', msgs.Objects, self.oois_cb)

        # Publisher
        self.trajectory_pub = rospy.Publisher(
            'ooi_trajectory', msgs.Trajectory)
        self.oois_pub = rospy.Publisher('oois', msgs.Objects)

        # real members
        self.ooi_id = ""
        self.oois = []
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
        while self.data_changed and not rospy.is_shutdown():
            rospy.sleep(.1)

        with self.trajectory_lock:
            if self.trajectory:
                msg = util.create_trajectory_msg(self.ooi_id, self.trajectory)
                self.trajectory_pub.publish(msg)

                get_ooi(self.oois, self.ooi_id)['body'].X = self.trajectory[-1]
                msg = util.create_oois_msg(self.oois)
                self.oois_pub.publish(msg)

                del self.trajectory[:]
                self.trajectory = []

                self.send = False
            else:
                rospy.loginfo("empty trajectory")

        self.server.set_succeeded()

    def percept_cb(self, data):
        if data.changed:
            for body_str in data.bodies:
                body = util.parse_body_msg(body_str)
                if body.name != self.ooi_id:
                    continue
                with self.trajectory_lock:
                    self.trajectory.append(corepy.Transformation(body.X))

        self.data_changed = data.changed

    def preempt_cb(self):
        self.server.set_preempted()

    def oois_cb(self, msg):
        self.oois = util.parse_oois_msg(msg)

    def ooi_id_cb(self, msg):
        self.ooi_id = msg.id


def main():
    rospy.init_node('tcr_sas_observe_ooi_traj')
    server = ObserveOOITrajActionServer('observe_ooi_traj')

if __name__ == '__main__':
    main()
    rospy.spin()

#!/usr/bin/env python
# encoding: utf-8

import roslib

roslib.load_manifest('the_curious_robot')
roslib.load_manifest('actionlib')
import rospy
import threading

from actionlib import SimpleActionServer
import the_curious_robot.msg as msgs
import corepy
import util
import require_provide as rp


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
            data_class=rosors.objects,
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
        while self.data_changed and not rospy.is_shutdown():
            rospy.sleep(.1)

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
                    body_msg = rospy.ServiceProxy("/world/bodies",
                            rosors.msgs.Object(obj))
                    body = msg_to_ors_body(body_msg)
                    self.trajectory.append(corepy.Transformation(body.X))


    def preempt_cb(self):
        self.server.set_preempted()


    def ooi_id_cb(self, msg):
        self.ooi_id = msg.id


def main():
    rospy.init_node('tcr_sas_observe_ooi_traj')
    server = ObserveOOITrajActionServer('observe_ooi_traj')

if __name__ == '__main__':
    main()
    rospy.spin()

#!/usr/bin/env python

# ROS
import roslib
roslib.load_manifest('actionlib')
from actionlib import SimpleActionServer
# MLR
roslib.load_manifest('the_curious_robot')
import require_provide as rp
import rosors.srv
import rospy
import the_curious_robot as tcr
# python std
import random
from timer import Timer


#########################################################################
# different strategies for selecting OOIs
#########################################################################
#
# to add new strategies add a method that takes the the list of oois. the
# `select_ooi` in `PickOOIActionServer` must be set to this method.
#
# TODO selecting the strategy should be done via dynamic reconfigure.

def _strategy_random_select(oois):
    """Select a random object of all possibes objects."""
    ooi = random.choice(oois)
    ooi_id_msg = tcr.msg.ObjectID()
    ooi_id_msg.id = ooi
    rospy.logdebug(ooi)
    return ooi_id_msg


class _strategy_sequential_select():
    def __init__(self):
        self.ooi_index = 0

    def __call__(self, oois):
        ooi_id_msg = tcr.msg.ObjectID()
        ooi_id_msg.id = oois[self.ooi_index]
        self.ooi_index = (self.ooi_index + 1) % len(oois)
        return ooi_id_msg


def _strategy_door_frame_top(oois):
    """Always go for the door1-door."""
    ooi_id_msg = tcr.msg.ObjectID()
    ooi_id_msg.id = 4
    return ooi_id_msg


def _strategy_select_shape_with_index(oois, index=5):
    """
    Pick a shape with the given index.

    4: top door frame
    5: door_door
    """
    ooi_id_msg = tcr.msg.ObjectID()
    ooi_id_msg.id = index
    return ooi_id_msg


#########################################################################
class PickOOIActionServer:

    def __init__(self, name):
        # right now we don't need the world_belief, that might change
        # self.world_belief_sub = rospy.Subscriber ...

        # Services
        self.request_all_shapes = rospy.ServiceProxy('/world/shapes',
                                                     rosors.srv.Shapes)
        # Publisher
        self.ooi_id_pub = rospy.Publisher('ooi_id', tcr.msg.ObjectID)

        # Actionlib Server
        self.server = SimpleActionServer(
            name,
            tcr.msg.PickOOIAction,
            execute_cb=self.execute,
            auto_start=False
        )
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        # Select the exploration strategies
        # self.select_ooi = _strategy_random_select
        # this is a class and must be initiated; does it help if we use a
        # closuer?
        self.select_ooi = _strategy_sequential_select()
        # self.select_ooi = _strategy_door_frame_top
        # self.select_ooi = _strategy_select_shape_with_index

        self.oois = None
        rp.Provide("PickOOI")

    def execute(self, msg):
        # We assume that we "see" all shapes from the beginning and the number
        # does not change. Therfore, we only request it once.

        with Timer("PICK: initial if", rospy.logwarn):
            if self.oois is None:
                all_shapes_msg = self.request_all_shapes(with_mesh=False)
                self.oois = [shape.index for shape in all_shapes_msg.shapes
                             if shape.name != "base"]

        # select an ooi
        with Timer("PICK: select ooi", rospy.logwarn):
            ooi_id_msg = self.select_ooi(self.oois)

        with Timer("PICK: publish", rospy.logwarn):
            self.ooi_id_pub.publish(ooi_id_msg)

        self.server.set_succeeded()

    def preempt_cb(self):
        self.server.set_preempted()


def main():
    rospy.init_node('tcr_sas_pick_ooi')
    PickOOIActionServer('pick_ooi')


if __name__ == '__main__':
    main()
    rospy.spin()

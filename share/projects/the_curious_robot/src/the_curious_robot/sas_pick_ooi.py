#!/usr/bin/env python
# encoding: utf-8

# ROS
import roslib
roslib.load_manifest('actionlib')
from actionlib import SimpleActionServer
# MLR
roslib.load_manifest('the_curious_robot')
import require_provide as rp
from the_curious_robot import srv
import the_curious_robot as tcr
import rosors.srv
import rospy
from timer import Timer
# python std
import random
import collections
import numpy as np


###############################################################################
# different strategies for selecting OOIs
###############################################################################
# To add new strategies Create a class that inherits from SelectionStrategy
# and implement all function.
#
# TODO selecting the strategy should be done via dynamic reconfigure.
class SelectionStrategy(object):
    """ABC for every SelectionStrategy."""
    def execute(self, oois, entropies):
        raise NotImplementedError()


###############################################################################
class StrategyRandomSelect(SelectionStrategy):
    def execute(self, oois, entropies):
        """Select a random object of all possibes objects."""
        rospy.loginfo("Selection strategy: RANDOM")
        ooi = random.choice(oois)
        ooi_id_msg = tcr.msg.ObjectID()
        ooi_id_msg.id = ooi
        rospy.logdebug(ooi)
        return ooi_id_msg


class StrategySequentialSelect(SelectionStrategy):
    def __init__(self):
        self.ooi_index = 15
        self.ooi_index = 32
        self.ooi_index = 0

    def execute(self, oois, entropies):
        rospy.loginfo("Selection strategy: SEQUENTIAL")
        ooi_id_msg = tcr.msg.ObjectID()
        ooi_id_msg.id = oois[32 + self.ooi_index]
        # self.ooi_index = (self.ooi_index + 1) % len(oois)
        self.ooi_index = (self.ooi_index + 1) % 2
        return ooi_id_msg


class StrategyDoorFrameTop(SelectionStrategy):
    def execute(self, oois, entropies):
        """Always go for the door1-door."""
        rospy.loginfo("Selection strategy: FRAME_TOP")
        ooi_id_msg = tcr.msg.ObjectID()
        ooi_id_msg.id = 4
        return ooi_id_msg


class StrategySelectShapeWithIndex(SelectionStrategy):
    def __init__(self):
        self.index = 5

    def execute(self, oois, entropies):
        """
        Pick a shape with the given index.

        4: top door frame
        5: door_door
        """
        rospy.loginfo("Selection strategy: SHAPE WITH ID")
        ooi_id_msg = tcr.msg.ObjectID()
        ooi_id_msg.id = self.index
        return ooi_id_msg


class StrategySelectEntropy(SelectionStrategy):
    def __init__(self, selection_type):
        self.selection_type = selection_type

    def execute(self, oois, entropies):
        rospy.loginfo("Selection strategy: ENTROPY with type %s",
                      self.selection_type)

        if self.selection_type == "sum":
            ooi, entropy = max(entropies.iteritems(),
                               key=lambda key_ent: sum(key_ent[1]))
            entropy = sum(entropy)

        elif self.selection_type == "mean":
            ooi, entropy = max(entropies.iteritems(),
                               key=lambda key_ent: np.mean(key_ent[1]))
            entropy = np.mean(entropy)

        elif self.selection_type == "max":
            ooi, entropy = max(entropies.iteritems(),
                               key=lambda key_ent: max(key_ent[1]))
            entropy = max(entropy)

        else:
            print "NOT HANDLED"
            raise NotImplementedError()

        rospy.loginfo("Selecting %d with H=%f", ooi, entropy)

        return ooi


#########################################################################
class PickOOIActionServer(object):

    def __init__(self, name):
        # right now we don't need the world_belief, that might change
        # self.world_belief_sub = rospy.Subscriber ...

        # Services
        self.request_entropy = rospy.ServiceProxy("/belief/entropy/",
                                                  srv.Entropy)
        self.request_all_shapes = rospy.ServiceProxy('/world/shapes',
                                                     rosors.srv.Shapes)
        # Publisher
        self.ooi_id_pub = rospy.Publisher('ooi_id', tcr.msg.ObjectID)

        # Actionlib Server
        self.server = SimpleActionServer(name,
                                         tcr.msg.PickOOIAction,
                                         execute_cb=self.execute,
                                         auto_start=False)
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        # Select the exploration strategies
        self.selection_strategy = StrategyRandomSelect()
        # self.selection_strategy = StrategySequentialSelect()
        # self.selection_strategy = StrategyDoorFrameTop()
        # self.selection_strategy = StrategySelectShapeWithIndex()
        # self.selection_strategy = StrategySelectEntropy("max")

        self.possible_oois = None
        rp.Provide("PickOOI")

    def execute(self, msg):
        # We assume that we "see" all shapes from the beginning and the number
        # does not change. Therfore, we only request it once.

        if self.possible_oois is None:
            with Timer("PICK: initial if", rospy.logdebug):
                all_shapes_msg = self.request_all_shapes(with_mesh=False)
                self.possible_oois = [shape.index
                                      for shape in all_shapes_msg.shapes
                                      if shape.name not in ["base", "robot"]]

        # select an ooi
        with Timer("PICK: select ooi", rospy.logdebug):

            # save entropy info in a dict of lists: {key: [ent1, ...], ...}
            response = self.request_entropy()
            entropies = collections.defaultdict(list)
            for id_, entropy in zip(response.shape_ids, response.entropies):
                entropies[id_].append(entropy)
            rospy.logdebug(entropies)

            ooi_id_msg = self.selection_strategy.execute(self.possible_oois,
                                                         entropies)

        with Timer("PICK: publish", rospy.logdebug):
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

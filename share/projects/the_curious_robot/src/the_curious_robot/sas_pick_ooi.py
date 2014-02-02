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
from the_curious_robot import strategies
import rosors.srv
import rospy
from timer import Timer
# python std
import random
import collections
import numpy as np


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
        all_strategies = {
            "random": (strategies.StrategyRandomSelect, None),
            "sequential": (strategies.StrategySequentialSelect, None),
            "door_frame": (strategies.StrategyDoorFrameTop, None),
            "shape_id": (strategies.StrategySelectShapeWithIndex, None),
            "entropy_sum": (strategies.StrategySelectEntropy, "sum"),
            "entropy_max": (strategies.StrategySelectEntropy, "max"),
            "entropy_mean": (strategies.StrategySelectEntropy, "mean"),
        }
        strategy_name = rospy.get_param("strategy_name", "random")
        strategy, parameter = all_strategies[strategy_name]
        if parameter:
            self.selection_strategy = strategy(parameter)
        else:
            self.selection_strategy = strategy()

        self.possible_oois = None
        rp.Provide("PickOOI")

    def get_agent_shapes(self):
        req = rosors.srv.ShapesRequest()
        req.agent_number = 0
        req.agent_number_set = True
        shapes = self.request_all_shapes(req)
        return [shape.index for shape in shapes.shapes]

    def execute(self, msg):
        # We assume that we "see" all shapes from the beginning and the number
        # does not change. Therfore, we only request it once.

        if self.possible_oois is None:
            with Timer("PICK: initial if", rospy.logdebug):
                all_shapes_msg = self.request_all_shapes(
                    with_mesh=False, agent_number_set=False)
                self.possible_oois = [shape.index
                                      for shape in all_shapes_msg.shapes
                                      if shape.index not in range(34, 85)]
                print self.possible_oois

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

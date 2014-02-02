# ROS
import roslib
roslib.load_manifest('the_curious_robot')
import rospy
import the_curious_robot as tcr
import numpy as np

# python std
import random


###############################################################################
# different strategies for selecting OOIs
##############################################################################
# To add new strategies Create a class that inherits from SelectionStrategy
# and implement all function.
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
        self.ooi_index = 0

    def execute(self, oois, entropies):
        rospy.loginfo("Selection strategy: SEQUENTIAL")
        ooi_id_msg = tcr.msg.ObjectID()
        ooi_id_msg.id = oois[self.ooi_index]
        # self.ooi_index = (self.ooi_index + 1) % len(oois)
        self.ooi_index = (self.ooi_index + 1) % len(oois)
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

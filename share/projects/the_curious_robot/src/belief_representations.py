"""
This file contains various representations of different part of the belief.
"""
import scipy.stats as ss


###############################################################################
class ShapeBelief(object):
    """
    ShapeBelief is a container for all beliefs about an shape.
    """
    def __init__(self):
        # members / sub beliefs
        self.object_type = ObjectTypeHypo()
        self.joint = None  # JointBelief()
        # TODO add more

    def __str__(self):
        result = ""
        for attr_name in vars(self):
            attr_val = str(getattr(self, attr_name))
            result += "%s: %s\n" % (attr_name, attr_val)
        return result


###############################################################################
class JointBelief(object):
    def __init__(self):
        pass

    def __str__(self):
        result = ""
        # TODO fill me
        return result


###############################################################################
class ObjectTypeHypo():
    """
    ObjectType represents the probability that an object has a certain type.

    Each object in the world can be either STATIC or FREE. We use a beta
    distribution to model the probability for the object types.
    """
    STATIC = 0
    FREE = 1

    def __init__(self):
        # uninformed prior for beta distribution
        self.alpha = 1
        self.beta = 1

    def update(self, OBJECT_TYPE):
        """Update the hypothesis. We can observe STATIC or FREE."""
        if OBJECT_TYPE == ObjectTypeHypo.STATIC:
            self.alpha += 1
        elif OBJECT_TYPE == ObjectTypeHypo.FREE:
            self.beta += 1
        else:
            raise TypeError("Type most be STATIC or FREE")

    def get_entropy(self):
        return ss.beta(self.alpha, self.beta).entropy()

    def __str__(self):
        dist = ss.beta(self.alpha, self.beta)
        return "Beta(alpha={}, beta={}) -> H={:1.3}".format(
            self.alpha, self.beta, dist.entropy()
        )

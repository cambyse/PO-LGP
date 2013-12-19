"""
This file contains various representations of different part of the belief.
"""
import scipy.stats as ss


###############################################################################
class ShapeBelief(object):
    """
    ShapeBelief is a container for all beliefs about an shape.
    """
    def __init__(self, belief_shape_id):
        self.belief_shape_id = belief_shape_id

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

    def is_static(self):
        """Return True iff the object is static."""
        return self.alpha >= self.beta

    def __str__(self):
        dist = ss.beta(self.alpha, self.beta)
        return "Beta(alpha={}, beta={}) -> H={:1.3}".format(
            self.alpha, self.beta, dist.entropy()
        )


###############################################################################
class JointBelief(object):
    """
    JointBelief stores all information about the joint.

    TODO: How should we update the likelihood we get from articulation?
    """

    # types of joints
    PRISMATIC = 1
    ROTATIONAL = 2

    def __init__(self):
        self.joint_type = None
        # store the rot/trans information in this dict
        self.values = {}
        self.loglikelihood = None

        # parameters for the beta distribution
        self._prismatic_count = 1  # alpha
        self._rotational_count = 1  # beta

    def __str__(self):
        dist = ss.beta(self._prismatic_count, self._rotational_count)
        return "Beta(prismatic={}, rotational={}) -> H={:1.3}".format(
            self._prismatic_count, self._rotational_count, dist.entropy()
        )

    def get_entropy(self):
        return ss.beta(self._prismatic_count, self._rotational_count).entropy()

    def update(self, JOINT_TYPE):
        """Update the hypothesis. We can observe STATIC or FREE."""
        if JOINT_TYPE == JointBelief.PRISMATIC:
            self._prismatic_count += 1
        elif JOINT_TYPE == JointBelief.ROTATIONAL:
            self._rotational_count += 1
        else:
            raise TypeError("Type most be STATIC or FREE")
        pass

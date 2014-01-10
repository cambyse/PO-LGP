"""
This file contains various representations of different part of the belief.
"""
import scipy.stats as ss
import collections


###############################################################################
class Annotation(collections.OrderedDict):
    """
    Annotation contains all beliefs over a shape.

    It's a mapping: ooi_id --> ShapeBelief

    It offers some convenient functions.
    """

    def __str__(self):
        result = "\n" + "=" * 60
        result += "\nbelief annotation (%d items)\n" % len(self)
        result += "=" * 60 + "\n"
        for key, value in self.iteritems():
            tmp = "ID %d\n%s\n" % (key, str(value))
            result += tmp
        result += "=" * 60
        return result

    def iter_entropy_normalized(self, property_):
        """
        Return the entropy normalized between 0 and 1.

        This is useful for visualization.
        """
        entropies = [getattr(shape_bel, property_).get_entropy()
                     for key, shape_bel in self.iteritems()]
        min_entropy = min(entropies)
        max_entropy = max(entropies)

        for shape_bel, entropy in zip(self.itervalues(), entropies):
            normalized = (entropy - min_entropy) / (max_entropy - min_entropy)
            yield shape_bel, normalized


###############################################################################
class ShapeBelief(object):
    """
    ShapeBelief is a container for all beliefs about one shape.
    """
    def __init__(self, belief_shape):
        self.belief_shape = belief_shape
        self.belief_shape_id = self.belief_shape.index

        # members / sub beliefs
        self.object_type = ObjectTypeHypo()
        self.joint = None  # JointBelief()
        # TODO add more

    def __str__(self):
        result = ""
        for attr_name in ["object_type", "joint"]:
            attr_val = str(getattr(self, attr_name))
            result += "    %s: %s\n" % (attr_name, attr_val)
        return result

    def __getstate__(self):
        """
        To avoid pickling SwigObjects we have to overwrite __getstate__ and
        ignore the varibales.
        """
        exclude_members = ["belief_shape"]
        return dict((k, v) for (k, v) in self.__dict__.iteritems()
                    if k not in exclude_members)


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
        self._static_count = 1
        self._free_count = 1

    def update(self, OBJECT_TYPE):
        """Update the hypothesis. We can observe STATIC or FREE."""
        if OBJECT_TYPE == ObjectTypeHypo.STATIC:
            self._static_count += 1
        elif OBJECT_TYPE == ObjectTypeHypo.FREE:
            self._free_count += 1
        else:
            raise TypeError("Type most be STATIC or FREE")

    def get_entropy(self):
        return ss.beta(self._static_count, self._free_count).entropy()

    def is_static(self):
        """Return True iff the object is static."""
        return self._static_count >= self._free_count

    def __str__(self):
        dist = ss.beta(self._static_count, self._free_count)
        return "Beta(static={}, free={}) -> H={}".format(
            self._static_count, self._free_count, dist.entropy()
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
        return "Beta(prismatic={}, rotational={}) -> H={}".format(
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

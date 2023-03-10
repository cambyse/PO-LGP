"""
This file contains various representations of different part of the belief.
"""
import math
import collections

import numpy as np
import scipy.stats as ss

import probdist
import util
from dynamic_model import DynamicModel


###############################################################################
Gauss = collections.namedtuple("Gauss", "mu sigma")


###############################################################################
class ObjectBel(probdist.CategoricalDist):
    """ObjectBel"""

    def __init__(self, name):
        super(ObjectBel, self).__init__({"static": 1, "movable": 1})
        self.name = name
        self.ors_shape = None

        self.joint_bel = JointBel(name)

    def update(self, observation):
        self.observe(observation)

        #if obs_obj_type == "movable":
        # self.joint_bel.update(obs_joint_type)

    def is_static(self):
        return self.prob("static") > self.prob("movable")

    def total_entropy(self):
        P_mo = self.prob("movable")
        P_nil = self.joint_bel.prob_cond("nil", ("nil", self.prob("static")))
        P_rot = self.joint_bel.prob_cond("rot", ("nil", self.prob("static")))
        P_pris = self.joint_bel.prob_cond("pris", ("nil", self.prob("static")))

        H_gauss = self.joint_bel.entropy_gauss()
        return sum([self.entropy(),
                    P_mo * self.joint_bel.entropy(),
                    P_nil * H_gauss["nil"],
                    P_rot * H_gauss["rot"],
                    P_pris * H_gauss["pris"]])

    def all_entropy_diff(self):
        P_mo = self.prob("movable")
        P_nil = self.joint_bel.prob_cond("nil", ("nil", self.prob("static")))
        P_rot = self.joint_bel.prob_cond("rot", ("nil", self.prob("static")))
        P_pris = self.joint_bel.prob_cond("pris", ("nil", self.prob("static")))

        h_change, _ = self.joint_bel.entropy_tmp()
        result = collections.OrderedDict([
            ("obj", self.entropy_diff()),
            ("joint", P_mo * self.joint_bel.entropy_diff()),
            ("nil", P_nil * h_change["nil"]),
            ("rot", P_rot * h_change["rot"]),
            ("pris", P_pris * h_change["pris"])
        ])
        return result

    def __str__(self):
        result = "{} {}\n  joint {}".format(
            self.name,
            str(super(ObjectBel, self).probs()),
            str(self.joint_bel))
        return result

    def __getstate__(self):
        """
        To avoid pickling certain stuff we have to overwrite __getstate__ and
        ignore the varibales.
        """
        exclude_members = ["ors_shape"]
        return dict((k, v)
                    for (k, v) in self.__dict__.iteritems()
                    if k not in exclude_members)


class JointBel(probdist.CategoricalDist):
    def __init__(self, name):
        super(JointBel, self).__init__({"nil": 1, "rot": 1, "pris": 1})
        self.name = name

        std = .69
        std = 1.09

        self.rot_model = DynamicModel()
        self.rot_limit_min = Gauss(-1, std)
        self.rot_limit_max = Gauss(2, std)
        self.rot_damping = Gauss(0, std)

        self.pris_model = DynamicModel()
        self.pris_limit_min = Gauss(-1, std)
        self.pris_limit_max = Gauss(2, std)
        self.pris_damping = Gauss(0, std)

        self.start = None

        self.nil = Gauss(0, .05)

        # Scaling and Gaussians
        precision = 0.1
        zero_entropy_of_gaussian = .248
        self.scaler = zero_entropy_of_gaussian / precision

        self.update_var = 1.
        self.noise = .01

    def update(self, obs_classification, articulation_response):
        self.observe(obs_classification)

        pose_projected = articulation_response.model.track.pose_projected
        projected_trajectory = np.ndarray([len(pose_projected), 3])
        if self.start is None:
            self.start = projected_trajectory[0, :]

        for i, p in enumerate(pose_projected):
            projected_trajectory[i, 0] = p.position.x
            projected_trajectory[i, 1] = p.position.y
            projected_trajectory[i, 2] = p.position.z

        dt = 1./60.  # Since we step the physic engine on our own, this is
                     # constant. Otherwise ask the perception system

        if obs_classification == "nil":
            return

        elif obs_classification == "pris":
            direction = np.array([0., 0., 0.])
            for param in articulation_response.model.params:
                if param.name == "prismatic_dir.x":
                    direction[0] = param.value
                if param.name == "prismatic_dir.y":
                    direction[1] = param.value
                if param.name == "prismatic_dir.z":
                    direction[2] = param.value
            trajectory_1D = util.prismatic_to_position(projected_trajectory,
                                                       direction, self.start)
            vel = trajectory_1D[1] - trajectory_1D[0]

            self.pris_model.add_observations(trajectory_1D, dt,
                                             trajectory_1D[0], vel)

            mu, sigma = self.pris_model.get_approx_gaussian("min_limit")
            self.pris_limit_min = Gauss(mu, sigma)
            mu, sigma = self.pris_model.get_approx_gaussian("max_limit")
            self.pris_limit_max = Gauss(mu, sigma)
            mu, sigma = self.pris_model.get_approx_gaussian("damping")
            self.pris_damping = Gauss(mu, sigma)

        elif obs_classification == "rot":
            axis_pos = np.array([0., 0., 0.])
            axis = np.array([0., 0., 1.])
            for param in articulation_response.model.params:
                if param.name == "rot_center.x":
                    axis_pos[0] = param.value
                elif param.name == "rot_center.y":
                    axis_pos[1] = param.value
                elif param.name == "rot_center.z":
                    axis_pos[2] = param.value
                #elif param.name == "rot_axis.x":
                    #axis[0] = param.value
                #elif param.name == "rot_axis.y":
                    #axis[1] = param.value
                #elif param.name == "rot_axis.z":
                    #axis[2] = param.value
                #elif param.name == "rot_radius":
                    #print("Radius: ", param.value)

            # print projected_trajectory
            # print axis
            # print axis_pos
            trajectory_angle = util.rotational_to_angle(projected_trajectory,
                                                        axis,
                                                        axis_pos,
                                                        self.start)

            vel = trajectory_angle[1] - trajectory_angle[0]

            # print trajectory_angle
            self.rot_model.add_observations(trajectory_angle, dt,
                                            trajectory_angle[0], vel)

            mu, sigma = self.rot_model.get_approx_gaussian("min_limit")
            print("Min. Limit: ", mu, sigma)
            self.rot_limit_min = Gauss(mu, sigma)
            mu, sigma = self.rot_model.get_approx_gaussian("max_limit")
            print("Max. Limit: ", mu, sigma)
            self.rot_limit_max = Gauss(mu, sigma)
            mu, sigma = self.rot_model.get_approx_gaussian("damping")
            print("Damping:    ", mu, sigma)
            self.rot_damping = Gauss(mu, sigma)

    def fwd_model(self, key):
        # forward model
        # update all three parameters of the joint
        for subname in ["_limit_max", "_limit_min", "_damping"]:
            fullname = key + subname
            gauss = getattr(self, fullname)
            distribution = ss.norm(gauss.mu, gauss.sigma)

            prior_var = distribution.var()
            post_std = math.sqrt((prior_var * self.update_var) /
                                 (prior_var + self.update_var))
            post_std += self.noise
            setattr(self, fullname, Gauss(gauss.mu, post_std))

    def entropy_tmp(self):
        """How do we want to calculate the change of entropy for the joint bel?

        - icrement the counter for each label in JointBel
        - update the appropiate Gaussian with a simple forward model::
            new gauss = old gauss * fake msmt (+ noise)

        nil only "collapses" the entropy of the other distributions. We don't
        have to calc the entropy here.

        """
        h_stats = {}
        HStat = collections.namedtuple("HStat", "cur exp P")
        h_change = {}

        for name in self:
            P = self.prob(name)
            P = 1

            if name == "nil":
                H = ss.norm.entropy(self.nil.mu, self.nil.sigma)
                H_expected = H
                h_stats[name] = HStat(float(H), float(H_expected), P)
                change = 0
                h_change[name] = change

            else:
                change = 0
                for subname in ["_limit_max", "_limit_min", "_damping"]:
                    fullname = name + subname
                    gauss = getattr(self, fullname)
                    distribution = ss.norm(gauss.mu, gauss.sigma)
                    H = distribution.entropy()

                    prior_var = distribution.var()
                    post_std = math.sqrt((prior_var * self.update_var) /
                                         (prior_var + self.update_var))
                    post_std += self.noise
                    H_expected = ss.norm.entropy(0, post_std)
                    h_stats[fullname] = HStat(float(H), float(H_expected), P)
                    change += (H - H_expected)
                h_change[name] = change

        return (h_change, h_stats)

    def entropy_gauss(self):
        """return dict for the unweighted entropy for ["pris", "rot", "nil"]"""
        result = {}

        gauss = self.rot_damping
        H = ss.norm.entropy(gauss.mu, gauss.sigma)
        gauss = self.rot_limit_max
        H += ss.norm.entropy(gauss.mu, gauss.sigma)
        gauss = self.rot_limit_min
        H += ss.norm.entropy(gauss.mu, gauss.sigma)
        result["rot"] = H

        gauss = self.pris_damping
        H = ss.norm.entropy(gauss.mu, gauss.sigma)
        gauss = self.pris_limit_max
        H += ss.norm.entropy(gauss.mu, gauss.sigma)
        gauss = self.pris_limit_min
        H += ss.norm.entropy(gauss.mu, gauss.sigma)
        result["pris"] = H

        gauss = self.nil
        H = ss.norm.entropy(gauss.mu, gauss.sigma)
        result["nil"] = H

        return result

    def __getstate__(self):
        """
        To avoid pickling certain stuff we have to overwrite __getstate__ and
        ignore the varibales.
        """
        exclude_members = ["rot_model", "pris_model"]
        return dict((k, v)
                    for (k, v) in self.__dict__.iteritems()
                    if k not in exclude_members)


###############################################################################
class Belief(collections.OrderedDict):
    """
    Belief contains all beliefs over the shapes shape.

    It's a mapping: ooi_id --> ObjBel

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

    def get_total_entropy(self):
        """Return the total weighted entropy for the belief.

        Use for visualization.
        """
        H_total = 0
        for obj in self.itervalues():
            P_static = obj.prob("static")
            P = {
                "static": P_static,
                "movable": obj.prob("movable"),
                "nil": obj.joint_bel.prob_cond("nil", ("nil", P_static)),
                "rot": obj.joint_bel.prob_cond("rot", ("nil", P_static)),
                "pris": obj.joint_bel.prob_cond("pris", ("nil", P_static))
            }
            H_gauss = obj.joint_bel.entropy_gauss()
            H_total += (obj.entropy()
                        + P["movable"] * obj.joint_bel.entropy()
                        + P["rot"] * H_gauss["rot"]
                        + P["pris"] * H_gauss["pris"]
                        + P["nil"] * H_gauss["nil"])
        return H_total

    # def iter_entropy_normalized(self, property_):
    #     """
    #     Return the entropy normalized between 0 and 1.

    #     This is useful for visualization.
    #     """
    #     entropies = [getattr(shape_bel, property_).get_entropy()
    #                  for key, shape_bel in self.iteritems()]
    #     # beta(2, 1).entropy() =! beta(1, 2).entropy()
    #     # The tiny difference fucks up the visualization, therefore we round
    #     # it.
    #     min_entropy = round(min(entropies), 5)
    #     max_entropy = round(max(entropies), 5)

    #     for shape_anno, entropy in zip(self.itervalues(), entropies):
    #        normalized = (entropy - min_entropy) / (max_entropy - min_entropy)
    #         if normalized == float('Inf'):
    #             normalized = 1.
    #         yield shape_anno.belief_shape, normalized

    def get_entropy(self):
        """
        Return a list of tuples of the form (shape_id, entropy).
        """
        result = []
        for k, obj_bel in self.iteritems():
            result.append((k, obj_bel.entropy()))

        return result


###############################################################################
# class ShapeBelief(object):
#     """
#     ShapeBelief is a container for all beliefs about one shape.
#     """
#     def __init__(self, belief_shape):
#         self.belief_shape = belief_shape
#         self.belief_shape_id = self.belief_shape.index

#         # members / sub beliefs
#         self.object_type = ObjectTypeHypo()
#         self.joint = None  # JointBelief()
#         # TODO add more

#         # list of all sub belief members is used to easily iterate over the
#         # belief and retrieve information
#         self._beliefs = ["object_type", "joint"]

#     def __str__(self):
#         result = ""
#         for attr_name in self._beliefs:
#             attr_val = str(getattr(self, attr_name))
#             result += "    %s: %s\n" % (attr_name, attr_val)
#         return result

#     def __getstate__(self):
#         """
#         To avoid pickling SwigObjects we have to overwrite __getstate__ and
#         ignore the varibales.
#         """
#         exclude_members = ["belief_shape"]
#         return dict((k, v) for (k, v) in self.__dict__.iteritems()
#                     if k not in exclude_members)


###############################################################################
# class ObjectTypeHypo(object):
#     """
#     ObjectType represents the probability that an object has a certain type.

#     Each object in the world can be either STATIC or FREE. We use a beta
#     distribution to model the probability for the object types.
#     """
#     STATIC = 0
#     FREE = 1

#     def __init__(self):
#         # uninformed prior for beta distribution
#         self._static_count = 1
#         self._free_count = 1

#     def update(self, OBJECT_TYPE):
#         """Update the hypothesis. We can observe STATIC or FREE."""
#         if OBJECT_TYPE == ObjectTypeHypo.STATIC:
#             self._static_count += 1
#         elif OBJECT_TYPE == ObjectTypeHypo.FREE:
#             self._free_count += 1
#         else:
#             raise TypeError("Type most be STATIC or FREE")

#     def get_entropy(self):
#         return ss.beta(self._static_count, self._free_count).entropy()

#     def is_static(self):
#         """Return True iff the object is static."""
#         return self._static_count >= self._free_count

#     def __str__(self):
#         dist = ss.beta(self._static_count, self._free_count)
#         return "Beta(static={}, free={}) -> H={}".format(
#             self._static_count, self._free_count, dist.entropy()
#         )


###############################################################################
# class JointBelief(object):
#     """
#     JointBelief stores all information about the joint.

#     TODO: How should we update the likelihood we get from articulation?
#     """

#     # types of joints
#     PRISMATIC = 1
#     ROTATIONAL = 2

#     def __init__(self):
#         self.joint_type = None
#         # store the rot/trans information in this dict
#         self.values = {}
#         self.loglikelihood = None

#         # parameters for the beta distribution
#         self._prismatic_count = 1  # alpha
#         self._rotational_count = 1  # beta

#     def __str__(self):
#         dist = ss.beta(self._prismatic_count, self._rotational_count)
#         return "Beta(prismatic={}, rotational={}) -> H={}".format(
#             self._prismatic_count, self._rotational_count, dist.entropy()
#         )

#     def get_entropy(self):
#       return ss.beta(self._prismatic_count, self._rotational_count).entropy()

#     def update(self, JOINT_TYPE):
#         """Update the hypothesis. We can observe STATIC or FREE."""
#         if JOINT_TYPE == JointBelief.PRISMATIC:
#             self._prismatic_count += 1
#         elif JOINT_TYPE == JointBelief.ROTATIONAL:
#             self._rotational_count += 1
#         else:
#             raise TypeError("Type most be STATIC or FREE")
#         pass

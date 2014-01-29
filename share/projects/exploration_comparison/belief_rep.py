import math

import collections
#import numpy as np
import scipy as sp
import scipy.stats as ss

import probdist


class ObjectBel(probdist.CategoricalDist):
    """ObjectBel"""

    def __init__(self, name):
        super(ObjectBel, self).__init__({"static": 1, "movable": 1})
        self.name = name

        self.joint_bel = JointBel(name)

    def observe(self, key):
        """ OVERWRITE function """
        obs_obj_type, obs_joint_type = key

        super(ObjectBel, self).observe(obs_obj_type)

        if obs_obj_type == "movable":
            self.joint_bel.observe(obs_joint_type)

    def __str__(self):
        result = "{} {} H={}\n  joint {}".format(
            self.name, self.probs(), self.entropy(), str(self.joint_bel))
        return result


class JointBel(probdist.CategoricalDist):
    def __init__(self, name):
        super(JointBel, self).__init__({"nil": 1, "rot": 1, "pris": 1})
        self.name = name

        self.rot_limit_min = sp.stats.norm(loc=-1, scale=2)
        self.rot_limit_max = sp.stats.norm(loc=2, scale=2)
        self.rot_damping = sp.stats.norm(loc=0, scale=.5)

        self.pris_limit_min = sp.stats.norm(loc=-1, scale=2)
        self.pris_limit_max = sp.stats.norm(loc=2, scale=2)
        self.pris_damping = sp.stats.norm(loc=0, scale=.5)

        self.nil = sp.stats.norm(loc=0, scale=.05)

        # TODO Scaling and Gaussians
        precision = 0.1
        zero_entropy_of_gaussian = .248
        self.scaler = zero_entropy_of_gaussian / precision

        self.update_var = .9
        self.noise = .01

    def observe(self, key):
        """ OVERWRITE function """
        super(JointBel, self).observe(key)

        if key == "nil":
            return

        # forward model
        # update all three parameters of the joint
        for subname in ["_limit_max", "_limit_min", "_damping"]:
            fullname = key + subname
            distribution = getattr(self, fullname)

            prior_var = distribution.var()
            post_std = math.sqrt((prior_var * self.update_var) /
                                 (prior_var + self.update_var))
            post_std += self.noise
            setattr(self, fullname, ss.norm(0, post_std))

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

            if name == "nil":
                H = self.nil.entropy()
                H_expected = H
                h_stats[name] = HStat(float(H), float(H_expected), P)
                change = 0
            else:
                change = 0
                for subname in ["_limit_max", "_limit_min", "_damping"]:
                    fullname = name + subname

                    distribution = getattr(self, fullname)
                    H = distribution.entropy()
                    # Simple Forward Model
                    # only the std determines the entropy for gaussians.
                    # therefore, only update the std
                    prior_var = distribution.var()
                    # prior_sigma = math.sqrt(prior_std)
                    post_std = math.sqrt((prior_var * self.update_var) /
                                         (prior_var + self.update_var))
                    post_std += self.noise
                    H_expected = ss.norm.entropy(0, post_std)

                    # TODO Scaling and Gaussians
                    h_stats[fullname] = HStat(float(H), float(H_expected), P)
                    change += P * (H - H_expected)

            h_change[name] = change

        return (h_change, h_stats)

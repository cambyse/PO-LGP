import copy
import math

import numpy as np
import scipy as sp
import scipy.stats as ss

import probdist


class ObjectBel(probdist.CategoricalDist):
    """ObjectBel"""

    def __init__(self, name):
        super(ObjectBel, self).__init__({"static": 1, "movable": 1})
        self.name = name

        self.joint_bel = JointBel(name)

    def H_diff_hier(self):
        """Return the expected change of entropy"""
        # print(self.H())
        change = []
        probs = self.probs()
        for k in self:
            distribution = copy.copy(self)
            distribution.observe(k)
            P = probs[k]
            H = distribution.H()
            # print("{} -- P: {:.2f} H: {:.3f} P*H: {:.3f}".format(
            #     distribution, P, H, P * H))
            change.append(P * H)
        return self.H() - sum(change)


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

        self.nil = sp.stats.norm(loc=0, scale=.1)

        # TODO Scaling and Gaussians
        precision = 0.1
        zero_entropy_of_gaussian = .248
        self.scaler = zero_entropy_of_gaussian / precision

    def H_tmp(self):
        """How do we want to calculate the change of entropy for the joint bel?

        - icrement the counter for each label in JointBel
        - update the appropiate Gaussian with a simple forward model::
            new gauss = old gauss * fake msmt (+ noise)

        nil only "collapses" the entropy of the other distributions. We don't
        have to calc the entropy here.

        """
        expected_change = {}

        update_sigma = .9
        noise = .01

        for name in self:
            print("calc {}".format(name))

            P = self.prob(name)

            if name == "nil":
                H = self.nil.entropy()
                expected_change[name] = (H, H, P)
            else:
                for subname in ["_limit_max", "_limit_min", "_damping"]:
                    fullname = name + subname

                    distribution = getattr(self, fullname)
                    # Simple Forward Model
                    # only the std determines the entropy for gaussians.
                    # therefore, only update the std
                    prior_std = distribution.std()
                    prior_sigma = math.sqrt(prior_std)
                    post_sigma = math.sqrt((prior_sigma * update_sigma) /
                                           (prior_sigma + update_sigma))
                    post_sigma += noise
                    post_sigma *= self.scaler
                    H_expected = ss.norm.entropy(0, post_sigma)
                    print("post sigma {}".format(post_sigma))

                    # TODO Scaling and Gaussians
                    expected_change[fullname] = (H, H_expected, P)

        return expected_change




















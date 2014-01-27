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

        self.rot = sp.stats.norm(loc=0, scale=1)
        self.pris = sp.stats.norm(loc=0, scale=1)
        self.nil = sp.stats.norm(loc=0, scale=.1)

        #
        precision = 0.1
        self.scaler = .25 / precision

    def H_tmp(self):
        """How do we want to calculate the change of entropy for the joint bel?

        - icrement the counter for each label in JointBel
        - update the appropiate Gaussian with a simple forward model::
            new gauss = old gauss * fake msmt (+ noise)

        nil only "collapses" the entropy of the other distributions. We don't
        have to calc the entropy here.

        """
        Ps = []
        prior_entropy = []
        expected_entropy = []
        expected_change = {}

        update_sigma = .9
        noise = .01

        for name in self:
            print("calc {}".format(name))

            distribution = getattr(self, name)
            if name == "nil":
                H = distribution.entropy()
            else:
                # Simple Forward Model
                # only the std determines the entropy for gaussians. therefore,
                # only update the std
                prior_std = distribution.std()
                prior_sigma = math.sqrt(prior_std)
                post_sigma = math.sqrt((prior_sigma * update_sigma) /
                                       (prior_sigma + update_sigma))
                post_sigma += noise
                post_sigma *= self.scaler
                H = ss.norm.entropy(0, post_sigma)
                print("post sigma {}".format(post_sigma))

            P = self.prob(name)
            Ps.append(P)
            expected_entropy.append(H)
            prior_entropy_scaled = ss.norm.entropy(0, post_sigma * self.scaler)
            prior_entropy.append(prior_entropy_scaled)
            expected_change[name] = P * (prior_entropy[-1]
                                         - expected_entropy[-1])

        # print("current_entropy", self.H())
        print("Ps", Ps)
        print("prior entropy", prior_entropy)
        print("exp_entropy", expected_entropy)

        return expected_change




















from __future__ import division
import numpy as np
import math
import copy


class CategoricalDist(dict):
    def __init__(self, init_dict):
        for k, v in init_dict.iteritems():
            self[k] = v

    def prob(self, key):
        return self[key] / self._normalizer()

    def prob_cond(self, key, cond):
        name, p = cond
        if name not in self:
            raise KeyError()

        if key == name:
            return p
        else:
            return (self.prob(key) / (1 - self.prob(name))) * (1-p)

    def probs(self):
        normalizer = self._normalizer()
        return {key: self[key] / normalizer for key in self}

    def iterprobs(self):
        for k, v in self.probs().iteritems():
            yield k, v

    def observe(self, key):
        self[key] += 1

    def _normalizer(self):
        return sum(self.itervalues())

    def entropy(self):
        return - sum(p * math.log(p) for _, p in self.iterprobs() if p > 0)

    def entropy_diff(self):
        """Return the expected change of entropy"""
        # print(self.H())
        change = []
        probs = self.probs()
        for k in self:
            distribution = copy.copy(self)
            distribution.observe(k)
            P = probs[k]
            H = distribution.entropy()
            # print("{} -- P: {:.2f} H: {:.3f} P*H: {:.3f}".format(
            #     distribution, P, H, P * H))
            change.append(P * H)
        return self.entropy() - sum(change)

    def __str__(self):
        result = "{} --> H={:.3f} H_exp_diff={:.3f}".format(
            self.probs(), self.entropy(), self.entropy_diff())
        return result

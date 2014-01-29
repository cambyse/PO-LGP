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


class PMF(dict):
    def update(self, likelihood):
        for key in likelihood:
            self[key] *= likelihood[key]
        self._normalize()

    def _normalize(self):
        normalizer = sum(self.itervalues())
        for key in self:
            self[key] /= normalizer

    def entropy(self):
        result = - sum([self[k] * math.log(self[k]) for k in self])
        return result

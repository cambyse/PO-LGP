import copy
import scipy as sp

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
        super(JointBel, self).__init__({"nil": 1, "rot": 0, "pris": 0})
        self.name = name

        self.rot = sp.stats.norm(loc=0, scale=1)
        self.pris = sp.stats.norm(loc=0, scale=1)
        self.nil = sp.stats.norm(loc=0, scale=.1)

    def H_tmp(self):
        print("P {}\nH: {:.2f} H_diff: {:.2f} H_rot: {} H_pris: {}\n". format(
            self.probs(),
            self.H(),
            self.H_diff(),
            self.rot.entropy(),
            self.pris.entropy()
        ))

        entropy_sum = 0
        for k, p in self.iterprobs():
            try:
                d = getattr(self, k)
                print(k, p, d.entropy(), p * d.entropy())
                entropy_sum += p * d.entropy()
            except AttributeError:
                print(k, "not found")
        print("entropy sum", entropy_sum)

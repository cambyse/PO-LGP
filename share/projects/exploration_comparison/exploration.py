import random
import scipy.stats as ss


class Door(object):
    def __init__(self, prob_open, name, verbose=False):
        self.p = prob_open
        self.name = name
        self.verbose = verbose

    def push(self):
        result = random.random() < self.p
        if self.verbose:
            print("{} (prob_open={}) {}".format(
                self.name, self.p,
                "opened" if result else "didn't open"
            ))
        return result


class DoorBel(object):
    """"DoorBel is a beta dist."""
    def __init__(self, name, opened=1, closed=1):
        self.name = name
        # uninformed prior
        self.opened = opened
        self.closed = closed

    def __str__(self):
        return "Beta(opened={}, closed={}) --> H={}".format(
            self.opened, self.closed, self.entropy()
        )

    def entropy(self):
        return ss.beta(self.opened, self.closed).entropy()

    def update(self, opened):
        if opened:
            self.opened += 1
        else:
            self.closed += 1

    def mean(self):
        return ss.beta.mean(self.opened, self.closed)

    def likelihood(self, loc):
        return ss.beta.pdf(loc, self.opened, self.closed)


def init(num=2):
    world = [Door(1, "d0"), Door(.2, "d1")]
    belief = [DoorBel(door.name) for door in world]
    return world, belief

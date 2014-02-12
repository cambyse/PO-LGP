import random
import math
import scipy.stats as ss


def random_select(belief):
    """Select objects randomly."""
    return random.randint(0, len(belief) - 1)


def strategy_entropy(belief):
    """Select the object with the highest entropy."""
    # select max entropy door
    max_ent = -999999
    for i, door in enumerate(belief):
        ent = door.entropy()
        if ent > max_ent:
            max_ent = ent
            idx = i
    return idx


def strategy_expected_change_of_entropy(belief):
    """Select the object with the highest expected change of entropy."""
    max_diff = -999999
    for i, bel in enumerate(belief):
        H = bel.entropy()
        # prob for outcome a
        P_a = bel.mean()
        # entropy for outcome a
        H_a = ss.beta.entropy(bel.opened + 1, bel.closed)
        # prob for outcome b
        P_b = 1 - P_a
        # entropy for outcome b
        H_b = ss.beta.entropy(bel.opened, bel.closed + 1)
        # new estimated entropy
        H_est = (P_a * H_a + P_b * H_b)
        # change of H
        diff = H - H_est

        if False:
            print("=================================")
            print("H:    ", H)
            print("P_a:  ", P_a)
            print("H_a:  ", H_a)
            print("P_b:  ", P_b)
            print("H_b:  ", H_b)
            print("H_est:", H_est)
            print("diff: ", diff)

        if diff > max_diff:
            max_diff = diff
            idx = i
    return idx


class strategy_sequential(object):
    def __init__(self):
        self.last_selection = -1

    def __call__(self, belief):
        self.last_selection = (self.last_selection + 1) % len(belief)
        return self.last_selection


class StrategyUCB(object):
    """Upper Confident Bound strategy.

    See Marc's lecture on "Bandits, Global Optimization, AL and Bayesian RL"

    """
    def __init__(self, not_pushed_yet):
        self.not_pushed_yet = not_pushed_yet

    def __call__(self, belief):
        # we push every object once to estimate the reward
        if self.not_pushed_yet:
            choice = random.choice(self.not_pushed_yet)
            self.not_pushed_yet -= choice
            return choice

        # push the "best" door
        else:
            # play machine i that maximizes: rewarad_i + srqt(2 * ln n / n_i)
            # reward = average reward of machine i

            # n = number of total rounds
            n = sum(bel.opened + bel.closed - 2 for bel in belief)
            ucb_score = []
            for bel in belief:
                # we could use the expected change of entropy here
                reward = bel.entropy()
                # n_i = how often did we push door i
                n_i = bel.opened + bel.closed - 2
                score = reward + math.sqrt(2 * math.log(n) / n_i)
                ucb_score.append(score)
            print(ucb_score)
            max(ucb_score)

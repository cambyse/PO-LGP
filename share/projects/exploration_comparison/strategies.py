import random
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

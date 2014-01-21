import random
import scipy.stats as ss
import numpy as np
import copy
import seaborn as sns
import matplotlib.pyplot as plt


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


###############################################################################
def init():
    world = [Door(1, "d0"), Door(.2, "d1")]
    belief = [DoorBel(door.name) for door in world]
    return world, belief


def init_all_openable(num=2):
    world = [Door(1, "d" + str(i)) for i in range(num)]
    belief = [DoorBel(door.name) for door in world]
    return world, belief


def init_all_openable_big():
    return init_all_openable(num=5)


def init_some_openable_big(num=5):
    world = [Door(i % 2, "d" + str(i)) for i in range(num)]
    belief = [DoorBel(door.name) for door in world]
    return world, belief


###############################################################################
def run_experiment(world, belief, select_strategy, num_interactions,
                   observation_model=None):
    """Run the given `select_strategy` for the given `num_inteactions` and
    return the complete histroy of the belief for further analysis.

    """
    bel_history = []
    for i in range(num_interactions):

        idx = select_strategy(belief)
        opened = world[idx].push()
        if observation_model:
            opened = observation_model(opened)
        belief[idx].update(opened)

        bel_history.append(copy.deepcopy(belief))
    return bel_history


def multi_run(f_init, strategy_name, select_strategy, num_interactions=100,
              runs=50, observation_model=None):
    """Execute multiple runs of the given strategy (with the given parameters).

    """
    print(strategy_name)

    # often used stuff
    world, belief = f_init()
    num_objects = len(world)
    names = [door.name for door in world]

    # Initialze arrays that we need throughout the runs
    entropy_obj = np.empty((runs, num_interactions, num_objects))
    entropy_total = np.zeros((runs, num_interactions))

    likelihood_obj = np.empty_like(entropy_obj)
    likelihood_total = np.ones_like(entropy_total)

    log_likelihood_obj = np.empty_like(entropy_obj)
    log_likelihood_total = np.ones_like(entropy_total)

    estimated_p_obj = np.empty_like(entropy_obj)
    real_p_obj = [np.full((1, num_interactions), world[i].p)
                  for i in range(num_objects)]

    error_obj = np.empty_like(entropy_obj)
    error_total = np.zeros_like(entropy_total)

    squared_error_obj = np.empty_like(entropy_obj)
    squared_error_total = np.zeros_like(entropy_total)

    summary = {}

    #========================================================================
    # RUN multiple times
    for run in range(runs):
        world, belief = f_init()
        bel = run_experiment(world, belief, select_strategy, num_interactions,
                             observation_model)

        #======================================================================
        # COLLECT AND SUMMARIZE DATA
        for i in range(num_objects):
            # Entropy over time
            entropy_obj[run, :, i] = np.array([b[i].entropy() for b in bel])
            entropy_total[run, :] += entropy_obj[run, :, i]

            # Esimate over time
            estimated_p_obj[run, :, i] = np.array([b[i].mean() for b in bel])

            # Error over time: abs(p_true, p_estimated)
            error_obj[run, :, i] = np.abs(
                real_p_obj[i] - estimated_p_obj[run, :, i])
            error_total[run, :] += error_obj[run, :, i]

            # Squared error over time
            squared_error_obj[run, :, i] = error_obj[run, :, i]**2
            squared_error_total[run, :] += squared_error_obj[run, :, i]

            # Likelihood over time
            likelihood_obj[run, :, i] = np.array([b[i].likelihood(world[i].p)
                                                  for b in bel])
            likelihood_total[run, :] *= likelihood_obj[run, :, i]

            log_likelihood_obj[run, :, i] = np.log(likelihood_obj[run, :, i])
            log_likelihood_total[run, :] *= log_likelihood_obj[run, :, i]

    summary["error"] = error_total.mean()
    summary["MSE"] = squared_error_total.mean()
    summary["entropy"] = entropy_total.mean()
    summary["likelihood"] = likelihood_total[:, -1].mean()
    summary["log_likelihood"] = log_likelihood_total[:, -1].mean()

    #========================================================================
    # PLOT strategies seperately
    fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(1, 6, figsize=(22, 4))
    colors = sns.color_palette()

    # entropy over time
    sns.tsplot(entropy_obj, condition=names, ax=ax1)
    sns.tsplot(entropy_total, condition=["toatl"], color=colors[3], ax=ax1)
    ax1.set_title("Entropy over time for " + strategy_name)

    # estimate over time
    sns.tsplot(estimated_p_obj, condition=names, ax=ax2)
    for i in range(num_objects):
        sns.tsplot(real_p_obj[i], color="grey", label="real p", ax=ax2)
    ax2.set_ylim((0, 1))
    ax2.set_title("Estimate over time for " + strategy_name)

    # error over time
    sns.tsplot(error_obj, condition=names, ax=ax3)
    sns.tsplot(error_total, condition=["total"], color=colors[3], ax=ax3)
    ax3.set_title("Error over time for " + strategy_name)

    # squared error over time
    sns.tsplot(squared_error_obj, condition=names, ax=ax4)
    sns.tsplot(squared_error_total, condition=["total"], color=colors[3],
               ax=ax4)
    ax4.set_title("Squared error over time for " + strategy_name)

    # likelihood over time
    sns.tsplot(likelihood_obj, condition=names, ax=ax5)
    sns.tsplot(likelihood_total, condition=["total"], color=colors[3], ax=ax5)
    ax5.set_title("Likelihood over time for " + strategy_name)

    # log likelihood over time
    sns.tsplot(log_likelihood_obj, condition=names, ax=ax6)
    sns.tsplot(log_likelihood_total, condition=["total"], color=colors[3],
               ax=ax6)
    ax6.set_title("Log-likelihood over time for " + strategy_name)

    return summary

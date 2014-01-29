from __future__ import division
import random
import scipy.stats as ss
import numpy as np
arr = np.array
import copy
import seaborn as sns
import matplotlib.pyplot as plt

###############################################################################
import belief
import world


###############################################################################
def init():
    n = 5
    world = [i for i in range(n)]
    bel = [belief.ObjectBel("obj" + str(i+1)) for i in range(n)]
    return world, bel



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

    expected_diff_H_obj = np.empty_like(entropy_obj)
    expected_diff_H_total = np.ones_like(entropy_total)

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
    data = {}

    #========================================================================
    # RUN multiple times
    for run in range(runs):
        world, belief = f_init()
        bel = run_experiment(world, belief, select_strategy, num_interactions,
                             observation_model)

        #======================================================================
        # COLLECT AND SUMMARIZE DATA
        for obj in range(num_objects):
            # Entropy over time
            entropy_obj[run, :, obj] = arr([b[obj].entropy() for b in bel])
            entropy_total[run, :] += entropy_obj[run, :, obj]

            # Expected change of entropy over time
            expected_diff_H_obj[run, :, obj] = arr([b[obj].exp_diff_H()
                                                    for b in bel])
            expected_diff_H_total[run, :] += expected_diff_H_obj[run, :, obj]
            # Esimate over time
            estimated_p_obj[run, :, obj] = arr([b[obj].mean() for b in bel])

            # Error over time: abs(p_true, p_estimated)
            error_obj[run, :, obj] = np.abs(real_p_obj[obj] -
                                            estimated_p_obj[run, :, obj])
            error_total[run, :] += error_obj[run, :, obj]

            # Squared error over time
            squared_error_obj[run, :, obj] = error_obj[run, :, obj]**2
            squared_error_total[run, :] += squared_error_obj[run, :, obj]

            # Likelihood over time
            likelihood_obj[run, :, obj] = arr([b[obj].likelihood(world[obj].p)
                                               for b in bel])
            likelihood_total[run, :] *= likelihood_obj[run, :, obj]

            log_likelihood_obj[run, :, obj] = arr([b[obj].logpdf(world[obj].p)
                                                   for b in bel])
            log_likelihood_total[run, :] *= log_likelihood_obj[run, :, obj]

    data["error"] = error_total
    data["MSE"] = squared_error_total
    data["entropy"] = entropy_total
    data["expected_diff_H"] = expected_diff_H_total
    data["likelihood"] = likelihood_total
    data["log_likelihood"] = log_likelihood_total

    summary["error"] = error_total.mean()
    summary["MSE"] = squared_error_total.mean()
    summary["entropy"] = entropy_total.mean()
    summary["expected_diff_H"] = expected_diff_H_total[:, -1].mean()
    summary["likelihood"] = likelihood_total[:, -1].mean()
    summary["log_likelihood"] = log_likelihood_total[:, -1].mean()

    #========================================================================
    # PLOT strategies seperately
    fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(1, 6, figsize=(22, 4))
    plt.suptitle(strategy_name, fontsize=14)
    colors = sns.color_palette()
    c3 = colors[3]

    # estimate over time
    ax = ax1
    sns.tsplot(estimated_p_obj, condition=names, ax=ax)
    for obj in range(num_objects):
        sns.tsplot(real_p_obj[obj], color="grey", label="real p", ax=ax)
    ax.set_ylim((0, 1))
    ax.set_title("Estimate over time")

    # error over time
    # ax = axX
    # sns.tsplot(error_obj, condition=names, ax=ax)
    # sns.tsplot(error_total, condition=["total"], color=c3, ax=ax)
    # ax.set_title("Error over time for " + strategy_name)

    # squared error over time
    ax = ax2
    sns.tsplot(squared_error_obj, condition=names, ax=ax)
    sns.tsplot(squared_error_total, condition=["total"], color=c3, ax=ax)
    ax.set_title("Squared error over time")

    # entropy over time
    ax = ax3
    sns.tsplot(entropy_obj, condition=names, ax=ax)
    sns.tsplot(entropy_total, condition=["toatl"], color=c3, ax=ax)
    ax.set_title("Entropy over time")

    # expected change of entropy
    ax = ax4
    sns.tsplot(expected_diff_H_obj, condition=names, ax=ax)
    ax.set_title("Expected change of Entropy")

    # likelihood over time
    ax = ax5
    sns.tsplot(likelihood_obj, condition=names, ax=ax)
    sns.tsplot(likelihood_total, condition=["total"], color=c3, ax=ax)
    ax.set_title("Likelihood over time")

    # log likelihood over time
    ax = ax6
    sns.tsplot(log_likelihood_obj, condition=names, ax=ax)
    sns.tsplot(log_likelihood_total, condition=["total"], color=c3, ax=ax)
    ax.set_title("Log-likelihood over time")

    return summary, data

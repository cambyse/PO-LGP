from __future__ import division

import copy
import collections
import random
from time import gmtime, strftime
import datetime

import scipy.stats as ss
import numpy as np
arr = np.array
import seaborn as sns
import matplotlib.pyplot as plt

###############################################################################
import belief_rep
import world_rep


###############################################################################
def init():
    n = 5
    world = [
        world_rep.Object("obj1", object_type="static", joint_type="nil"),
        world_rep.Object("obj2", object_type="movable", joint_type="pris"),
        world_rep.Object("obj3", object_type="movable", joint_type="pris"),
        world_rep.Object("obj4", object_type="movable", joint_type="rot"),
        world_rep.Object("obj5", object_type="movable", joint_type="rot"),
    ]
    belief = [belief_rep.ObjectBel("obj" + str(i + 1)) for i in range(n)]
    return world, belief


def init_wall_and_door():
    n = 2
    world = [
        world_rep.Object("obj1", object_type="static", joint_type="nil"),
        world_rep.Object("obj2", object_type="movable", joint_type="pris"),
    ]
    belief = [belief_rep.ObjectBel("obj" + str(i + 1)) for i in range(n)]
    return world, belief


###############################################################################
def plot_belief(belief):
    n = len(belief)
    width = .5
    ind = np.arange(n)

    # COLLECT DATA
    obj_names = [obj.name for obj in belief]
    n = len(belief)

    # probabilities
    P = collections.OrderedDict()
    P["static"] = arr([obj.prob("static") for obj in belief])
    P["movable"] = arr([obj.prob("movable") for obj in belief])
    P["nil"] = arr([obj.joint_bel.prob_cond("nil", ("nil", P["static"][i]))
                    for i, obj in enumerate(belief)])
    P["rot"] = arr([obj.joint_bel.prob_cond("rot", ("nil", P["static"][i]))
                    for i, obj in enumerate(belief)])
    P["pris"] = arr([obj.joint_bel.prob_cond("pris", ("nil", P["static"][i]))
                    for i, obj in enumerate(belief)])

    # entropy
    H = collections.OrderedDict()
    H["nil"] = arr([
        P["nil"][i] * float(obj.joint_bel.nil.entropy())
        for i, obj in enumerate(belief)
    ])
    H["obj"] = arr([obj.entropy() for obj in belief])
    H["joint"] = arr([
        P["movable"][i] * obj.joint_bel.entropy()
        for i, obj in enumerate(belief)
    ])
    H["rot_limit_min"] = arr([
        P["rot"][i] * float(obj.joint_bel.rot_limit_min.entropy())
        for i, obj in enumerate(belief)
    ])
    H["rot_limit_max"] = arr([
        P["rot"][i] * float(obj.joint_bel.rot_limit_max.entropy())
        for i, obj in enumerate(belief)
    ])
    H["rot_damping"] = arr([
        P["rot"][i] * float(obj.joint_bel.rot_damping.entropy())
        for i, obj in enumerate(belief)
    ])
    H["pris_limit_min"] = arr([
        P["pris"][i] * float(obj.joint_bel.pris_limit_min.entropy())
        for i, obj in enumerate(belief)
    ])
    H["pris_limit_max"] = arr([
        P["pris"][i] * float(obj.joint_bel.pris_limit_max.entropy())
        for i, obj in enumerate(belief)
    ])
    H["pris_damping"] = arr([
        P["pris"][i] * float(obj.joint_bel.pris_damping.entropy())
        for i, obj in enumerate(belief)
    ])
    differential_dists = ["nil",
                          "rot_limit_min", "rot_limit_max", "rot_damping",
                          "pris_limit_min", "pris_limit_max", "pris_damping"]
    # expected change of entropy
    Hd = collections.OrderedDict(
        [("nil", []), ("obj",  []), ("joint", []), ("rot", []), ("pris", [])]
    )
    for i, obj in enumerate(belief):
        ent_dict = obj.all_entropy_diff()
        for name in ["nil", "obj", "joint", "rot", "pris"]:
            Hd[name].append(ent_dict[name])

    # Hd["nil"] = arr([P["nil"][i] * obj.joint_bel.entropy_tmp()[0]["nil"]
    #                  for obj in belief])
    # Hd["obj"] = [obj.entropy_diff() for obj in belief]
    # Hd["joint"] = [P["movable"][i] * obj.joint_bel.entropy_diff()
    #                for i, obj in enumerate(belief)]
    # Hd["rot"] = arr([P["rot"][i] * obj.joint_bel.entropy_tmp()[0]["rot"]
    #                  for i, obj in enumerate(belief)])
    # Hd["pris"] = arr([P["pris"][i] * obj.joint_bel.entropy_tmp()[0]["pris"]
    #                   for i, obj in enumerate(belief)])

    # PLOT DATA
    fig, axes = plt.subplots(1, 5, figsize=(20, 4))
    colors = sns.color_palette("husl", 8)
    c_st = colors[0]
    c_mo = colors[1]
    c_rot = colors[2]
    c_pris = colors[3]
    c_misc1 = colors[6]
    c_misc2 = colors[7]
    c_misc = [c_misc1, c_misc2]

    # object bel
    ax = axes[0]
    ax.bar(ind, P["static"], width, color=c_st, label="static")
    ax.bar(ind, P["movable"], width, bottom=P["static"], color=c_mo,
           label="movable")
    # ax.bar(ind+width, H["obj"], width/4, color=c_misc, label="entropy")
    # ax.bar(ind+width+width/4, Hd["obj"], width/4, color=c_misc,
    #        label="exp H")

    ax.set_ylim([0, 1.6])
    ax.set_xticklabels(obj_names, ind)
    ax.set_xticks(ind + width/2)
    ax.legend(loc="upper left")
    ax.set_ylabel("P")
    ax.set_title("Probability for object type")

    # joint bel
    ax = axes[1]
    ax.bar(ind, P["nil"], width, color=c_st, label="nil")
    ax.bar(ind, P["pris"], width, bottom=P["nil"], color=c_pris,
           label="prismatic")
    ax.bar(ind, P["rot"], width, bottom=P["nil"]+P["pris"], color=c_rot,
           label="rotational")
    # ax.bar(ind+width, H["joint"], width/4, color=colors[4], label="entropy")
    # ax.bar(ind+width+width/4, Hd["joint"], width/4, color=colors[5],
    #        label="exp H")

    ax.set_ylim([0, 1.6])
    ax.set_xticklabels(obj_names, ind)
    ax.set_xticks(ind + width/2)
    ax.legend(loc="upper left")
    ax.set_ylabel("P")
    ax.set_title("Probability for joint type")

    # Entropy
    ax = axes[2]
    discrete = ["obj", "joint"]
    w = 1 / (len(discrete) + 1)
    for i, name in enumerate(discrete):
        ax.bar(ind + (i * w), H[name], w, color=c_misc[i % 2], label=name)

    ax.set_ylim([0, 1.6])
    ax.set_ylabel("Entropy")
    ax.set_xticklabels(obj_names, ind)
    ax.set_xticks(ind + width/2)
    ax.legend(loc="upper left")
    ax.set_title("Entropy discrete")

    # Entropy differential
    ax = axes[3]
    w = 1 / (len(differential_dists) + 1)
    for i, name in enumerate(differential_dists):
        ax.bar(ind + (i * w), H[name], w, color=colors[i], label=name)

    ax.set_ylim([-2, 6.])
    ax.set_ylabel("Entropy differential")
    ax.set_xticklabels(obj_names, ind)
    ax.set_xticks(ind + width/2)
    ax.legend(loc="upper left")
    ax.set_title("Entropy  differential")

    # # Entropy diff
    ax = axes[4]
    w = 1 / (len(Hd) + 1)
    for i, (name, val) in enumerate(Hd.iteritems()):
        # ax.bar(ind + i * w, Hd[name], w, label=name, color=colors[i])
        ax.bar(ind + i * w, val, w, label=name, color=colors[i])

    ax.set_ylabel("Expected change of entropy")
    ax.set_ylim([0, .4])
    ax.set_xticklabels(obj_names, ind)
    ax.set_xticks(ind + width/2)
    ax.legend(loc="upper left")
    ax.set_title("Expected change of entropy")

    return fig


###############################################################################
def run_experiment(world, belief, select_strategy, num_interactions,
                   observation_model=None, make_pics=False):
    """Run the given `select_strategy` for the given `num_inteactions` and
    return the complete histroy of the belief for further analysis.

    """
    bel_history = []
    for i in range(num_interactions):

        idx = select_strategy(belief)
        # print("selecting object {}".format(idx))
        observations = world[idx].interact()
        if observation_model:
            observations = observation_model(observations)
        belief[idx].update(observations)

        if make_pics:
            fig = plot_belief(belief)
            filename = datetime.datetime.now().strftime(
                "%Y-%m-%d_%H:%M:%S:%f.png")
            fig.savefig(filename)

        # for obj in belief:
        #     print(str(obj))
        # print("=" * 80)

        bel_history.append(copy.deepcopy(belief))
    return bel_history


def multi_run(f_init, select_strategy, num_interactions=100,
              runs=50, observation_model=None):
    """Execute multiple runs of the given strategy (with the given parameters).

    """
    strategy_name = select_strategy.name
    print(strategy_name)

    # often used stuff
    world, belief = f_init()
    num_objects = len(world)
    names = [str(door) for door in world]

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
    real_p_obj = np.empty_like(entropy_obj)
                #[np.full((1, num_interactions), world[i].p)
                #  for i in range(num_objects)]

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
        for i in range(num_objects):
            # Entropy over time
            entropy_obj[run, :, i] = arr([
                b[i].entropy()
                + b[i].joint_bel.entropy() +
                + b[i].joint_bel.prob("pris") * (
                    b[i].joint_bel.rot_limit_max.entropy()
                    + b[i].joint_bel.rot_limit_min.entropy()
                    + b[i].joint_bel.rot_damping.entropy())
                + b[i].joint_bel.prob("pris") * (
                    b[i].joint_bel.pris_limit_max.entropy()
                    + b[i].joint_bel.pris_limit_min.entropy()
                    + b[i].joint_bel.pris_damping.entropy())
                + b[i].joint_bel.prob("nil") * b[i].joint_bel.nil.entropy()
                for b in bel])
            entropy_total[run, :] += entropy_obj[run, :, i]

            # Expected change of entropy over time
            # expected_diff_H_obj[run, :, i] = arr([b[i].exp_diff_H()
            #                                         for b in bel])
            # expected_diff_H_total[run, :] += expected_diff_H_obj[run, :, i]
            # # Esimate over time
            # estimated_p_obj[run, :, i] = arr([b[i].mean() for b in bel])

            # # Error over time: abs(p_true, p_estimated)
            # error_obj[run, :, i] = np.abs(real_p_obj[i] -
            #                                 estimated_p_obj[run, :, i])
            # error_total[run, :] += error_obj[run, :, i]

            # # Squared error over time
            # squared_error_obj[run, :, i] = error_obj[run, :, i]**2
            # squared_error_total[run, :] += squared_error_obj[run, :, i]

            # # Likelihood over time
            # likelihood_obj[run, :, i] = arr([b[i].likelihood(world[i].p)
            #                                    for b in bel])
            # likelihood_total[run, :] *= likelihood_obj[run, :, i]

            # log_likelihood_obj[run, :, i] = arr([b[i].logpdf(world[i].p)
            #                                        for b in bel])
            # log_likelihood_total[run, :] *= log_likelihood_obj[run, :, i]

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

    return summary, data
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

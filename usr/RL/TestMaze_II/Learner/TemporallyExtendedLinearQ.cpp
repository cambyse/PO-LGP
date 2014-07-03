#include "TemporallyExtendedLinearQ.h"

#include "ConjunctiveAdjacency.h"
#include "../util/util.h"
#include "../util/QtUtil.h"
#include "../util/ProgressBar.h"

#warning exclude again
#include "../Maze/MazeObservation.h"
#include "../Maze/MazeAction.h"

#include <omp.h>
#define USE_OMP

#include <iomanip>

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using util::Range;
using util::INVALID;

using std::vector;
using std::map;
using std::dynamic_pointer_cast;
using std::cout;
using std::endl;
using std::get;
using std::make_tuple;
using std::tuple;

using arma::zeros;
using arma::eye;
using arma::solve;
using arma::as_scalar;
using arma::conv_to;
using arma::kron;

typedef TemporallyExtendedLinearQ TELQ;

TemporallyExtendedLinearQ::TemporallyExtendedLinearQ(std::shared_ptr<ConjunctiveAdjacency> N,
                                                     double d):
    TemporallyExtendedFeatureLearner(N), discount(d)
{
    // include only action features for t=0
    N_plus->set_t_zero_features(ConjunctiveAdjacency::ACTION);
    // set outcome to "action"
    set_outcome_type(OUTCOME_TYPE::ACTION);
}

double TELQ::get_action_value(const_instance_ptr_t ins, action_ptr_t act) const {
    // return zero if value cannot be computed
    if(feature_set.size()==0) {
        DEBUG_WARNING("Cannot compute action (no features)");
        return 0;
    }
    // get feature values
    row_vec_t feature_values(feature_set.size());
    int feature_idx = 0;
    for(f_ptr_t feature : feature_set) {
        feature_values(feature_idx) = feature->evaluate(ins,act,observation_space,reward_space);
        ++feature_idx;
    }
    // compute action value
    return as_scalar(feature_values*weights);
}

TELQ::action_ptr_t TELQ::get_action(const_instance_ptr_t ins) {
    vector<action_ptr_t> optimal_actions;
    double max_action_value = -DBL_MAX;
    DEBUG_OUT(4,"Instance " << ins);
    for(action_ptr_t act : action_space) {
        // compute action value
        double action_value = get_action_value(ins,act);
        DEBUG_OUT(4,"    " << act << " --> " << action_value);
        // update optimal actions
        if(action_value>max_action_value) {
            DEBUG_OUT(4,"    is optimal action");
            max_action_value = action_value;
            optimal_actions.assign(1,act);
        } else if(action_value==max_action_value) {
            DEBUG_OUT(4,"    added to optimal action");
            optimal_actions.push_back(act);
        }
    }
    assert(optimal_actions.size()>0);
    int random_idx = rand()%optimal_actions.size();
    return optimal_actions[random_idx];
}

double TELQ::run_policy_iteration() {
    int counter = 0;
    double old_TD_error = DBL_MAX;
    enum MODE { ANALYTICALLY, L1 };
    MODE mode = ANALYTICALLY;
    IF_DEBUG(1) { cout << "------------------------------BEGIN: Policy Iteration-----------------------------" << endl;}
    while(true) {
        update_policy();
        switch(mode) {
        case ANALYTICALLY:
            optimize_weights_Bellman_residual_error();
            break;
        case L1:
            optimize_weights_LBFGS();
            break;
        }
        double TD_error = get_TD_error();
        ++counter;
        IF_DEBUG(1) {
            cout << "    Iteration: " << counter << ", TD-error: " << TD_error << " ( delta = " << old_TD_error-TD_error << ")" << endl;
        }
        if(old_TD_error-TD_error>0) {
            old_TD_error = TD_error;
        } else {
            if(mode==ANALYTICALLY) {
                mode = L1;
                //------------------------//
#warning just one L1 step, no iteration
                optimize_weights_LBFGS();
                break;
                //-----------------------//
            } else {
                break;
            }
        }
    }
#warning update policy based on L1-regularized optimization??
    //update_policy();
    IF_DEBUG(1) { cout << "------------------------------END: Policy Iteration-------------------------------" << endl;}
    return old_TD_error;
}

double TELQ::get_TD_error() {
    update_c_rho_L();
    return as_scalar(c + 2*rho.t()*weights + weights.t()*L*weights);
}

void TELQ::print_training_data() const {
    int data_idx = 0;
    int episode_idx = 0;
    for(const_instance_ptr_t episode : instance_data) {
        cout << "Episode " << episode_idx << endl;
        for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {
            cout << ins << endl;
            for(action_ptr_t act : action_space) {
                cout << "    " << act << " --> " << get_action_value(ins,act) << endl;
            }
            ++data_idx;
        }
        ++episode_idx;
    }
}

bool TELQ::update() {
    if(data_changed) {
        update_rewards_and_data_indices();
    }
    bool performed_update = TemporallyExtendedFeatureLearner::update();
    if(performed_update) {
        need_to_update_c_rho_L = true;
    }
    return performed_update;
}

void TELQ::update_policy() {

    // resize but remember old
    auto old_policy = policy;
    auto old_policy_indices = policy_indices;
    policy.resize(number_of_data_points);
    policy_indices.resize(number_of_data_points);

    // random policy if value function cannot be computed
    if(weights.size()==0) {
        for(int data_idx : Range(number_of_data_points)) {
            policy[data_idx] = action_space->random_element();
            unsigned int action_idx = 0;
            for(action_ptr_t act_compare : action_space) {
                if(act_compare==policy[data_idx]) {
                    policy_indices[data_idx] = action_idx;
                    break;
                }
                ++action_idx;
            }
            assert(action_idx<action_space->space_size());
        }
    } else {

        // make sure all data are up-to-date
        update();

        // choose maximum value action efficiently using precomputed values
        for(int data_idx : Range(number_of_data_points)) {
            row_vec_t action_values = weights.t()*F_matrices[data_idx];
            int outcome_idx = 0;
            vector<action_ptr_t> optimal_actions;
            vector<int> optimal_action_indices;
            double max_action_value = -DBL_MAX;
            for(action_ptr_t act : action_space) {
                if(action_values(outcome_idx)>max_action_value) {
                    max_action_value = action_values(outcome_idx);
                    optimal_actions.assign(1,act);
                    optimal_action_indices.assign(1,outcome_idx);
                } else if(action_values(outcome_idx)==max_action_value) {
                    optimal_actions.push_back(act);
                    optimal_action_indices.push_back(outcome_idx);
                }
                ++outcome_idx;
            }
            assert(optimal_actions.size()>0);

#warning this is not settled
            // randomization might counteract convergence of policy iteration, could
            // use fixed tie-breaking instead
            int random_idx = rand()%optimal_actions.size();
            policy[data_idx] = optimal_actions[random_idx];
            policy_indices[data_idx] = optimal_action_indices[random_idx];
        }

        // choose maximum value action the stupid costly way
        // int data_idx = 0;
        // for(const_instance_ptr_t episode : instance_data) {
        //     for(const_instance_ptr_t ins_t=episode->const_first(); ins_t!=INVALID; ++ins_t) {
        //         action_ptr_t action = get_action(ins_t);
        //         policy[data_idx] = action;
        //         policy_indices[data_idx] = action->index();
        //         IF_DEBUG(3) {
        //             DEBUG_OUT(0,ins_t);
        //             for(action_ptr_t act : action_space) {
        //                 DEBUG_OUT(0,"    " << act << " --> " << get_action_value(ins_t,act));
        //             }
        //             DEBUG_OUT(0,"    ==> " << action);
        //             DEBUG_OUT(0,"");
        //         }
        //         ++data_idx;
        //     }
        // }

    }

    // set flag if changed
    if(policy_indices!=old_policy_indices) {
        IF_DEBUG(1) {
            if(policy==old_policy) {
                DEBUG_DEAD_LINE;
            }
        }
        need_to_update_c_rho_L = true;
    } else {
        IF_DEBUG(1) {
            if(policy!=old_policy) {
                DEBUG_DEAD_LINE;
            }
        }
    }
}

void TELQ::update_rewards_and_data_indices() {
    IF_DEBUG(1) {
        cout << "Update rewards and data-indices" << endl;
    }
    rewards_and_data_indices.clear();
    int data_idx = 0;
    for(const_instance_ptr_t episode : instance_data) {
        for(const_instance_ptr_t ins_t=episode->const_first(); ins_t->const_next()!=INVALID; ++ins_t) {
            rewards_and_data_indices.push_back(tuple<double,int>(ins_t->reward->get_value(),data_idx));
            ++data_idx;
        }
        // last instance in each episode is skipped
        ++data_idx;
    }
    assert(number_of_data_points=rewards_and_data_indices.size()+instance_data.size());
}

TELQ::action_ptr_t TELQ::optimal_2x2_policy(const_instance_ptr_t ins) const {
    action_ptr_t action;
    if(ins->observation==MazeObservation(2,2,0,0)) {
        action = action_ptr_t(new MazeAction("d"));
    } else if(ins->observation==MazeObservation(2,2,1,1)) {
        action = action_ptr_t(new MazeAction("u"));
    } else if(ins->observation==MazeObservation(2,2,0,1)) {
        if(ins->const_prev()->observation==MazeObservation(2,2,0,0)) {
            action = action_ptr_t(new MazeAction("r"));
        } else {
            action = action_ptr_t(new MazeAction("u"));
        }
    } else if(ins->observation==MazeObservation(2,2,1,0)) {
        if(ins->const_prev()->observation==MazeObservation(2,2,0,0)) {
            action = action_ptr_t(new MazeAction("d"));
        } else {
            action = action_ptr_t(new MazeAction("l"));
        }
    } else {
        DEBUG_ERROR("Unexpected observation unsing default action 'stay'");
        action = action_ptr_t(new MazeAction("s"));
    }
    return action;
}

void TELQ::set_optimal_2x2_policy() {
    // set flag
    need_to_update_c_rho_L = true;
    // resize
    policy.resize(number_of_data_points);
    policy_indices.resize(number_of_data_points);
    // set optimal policy
    int data_idx = 0;
    for(const_instance_ptr_t episode : instance_data) {
        for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {
            action_ptr_t action = optimal_2x2_policy(ins);
            int outcome_idx = 0;
            bool found = false;
            for(action_ptr_t act : action_space) {
                if(act==action) {
                    found = true;
                    policy[data_idx] = action;
                    policy_indices[data_idx] = outcome_idx;
                    break;
                }
                ++outcome_idx;
            }
            if(!found) {
                DEBUG_ERROR("Unexpected action using 'stay' and idx=0");
                policy[data_idx] = action_ptr_t(new MazeAction("s"));
                policy_indices[data_idx] = 0;
            }
            ++data_idx;
            // print
            // cout << "Instance: " << ins->observation << endl;
            // cout << "          " << ins->const_prev()->observation << endl;
            // cout << "          --> " << action << endl;
        }
    }
}

void TELQ::update_c_rho_L() {
    // only update if necessary
    if(!need_to_update_c_rho_L) {
        return;
    }

    // first update everything else
    update();

    // update
    int feature_n = feature_set.size();
    c = 0;
    rho.zeros(feature_n);
    L.zeros(feature_n,feature_n);
    int progress_counter = 0;
    IF_DEBUG(1) { ProgressBar::init("Updating c-rho-L:          "); }

#ifdef USE_OMP
#pragma omp parallel
    {
        int nr_threads = omp_get_num_threads();
        vector<double> c_update(nr_threads,0);
        vector<col_vec_t> rho_update(nr_threads, zeros<col_vec_t>(rho.size()));
        vector<f_mat_t> L_update(nr_threads,zeros<f_mat_t>(L.n_rows,L.n_cols));
#pragma omp for schedule(static) collapse(1)
        for(unsigned int idx = 0; idx<rewards_and_data_indices.size(); ++idx) {
            tuple<double,int> reward_and_data_index = rewards_and_data_indices[idx];
            double r_t = get<0>(reward_and_data_index);
            int t_idx = get<1>(reward_and_data_index);
            col_vec_t phi = (col_vec_t)(discount * F_matrices[t_idx+1].col(policy_indices[t_idx]) -
                                        F_matrices[t_idx].col(outcome_indices[t_idx]));
            int thread_nr = omp_get_thread_num();
            c_update[thread_nr] = c_update[thread_nr] + pow(r_t,2);
            rho_update[thread_nr] = rho_update[thread_nr] + r_t * phi;
            L_update[thread_nr] = L_update[thread_nr] + kron(phi,phi.t());
#pragma omp critical
            {
                ++progress_counter;
                IF_DEBUG(1) { ProgressBar::print(progress_counter,number_of_data_points); }
            }
        } // end OMP for
#pragma omp critical
        {
            for(int thread_idx=0; thread_idx<nr_threads; ++thread_idx) {
                c = c + c_update[thread_idx];
                rho = rho + rho_update[thread_idx];
                L = L + L_update[thread_idx];
            }
        }
    } // end OMP parallel

#else

    for(auto reward_and_idx : rewards_and_data_indices) {
        double r_t = get<0>(reward_and_idx);
        int t_idx = get<1>(reward_and_idx);
        //DEBUG_OUT(0,"T-idx: " << t_idx << " (" << policy_indices.size() << "/" << outcome_indices.size() << "/" << F_matrices.size() << ")");
        col_vec_t phi = (col_vec_t)(discount * F_matrices[t_idx+1].col(policy_indices[t_idx]) -
                                    F_matrices[t_idx].col(outcome_indices[t_idx]));
        c = c + pow(r_t,2);
        rho = rho + r_t * phi;
        L = L + kron(phi,phi.t());
        ++progress_counter;
        ++t_idx;
        IF_DEBUG(1) { ProgressBar::print(progress_counter,number_of_data_points); }
    }

#endif

    IF_DEBUG(1) { ProgressBar::terminate(); }
    double normalization = rewards_and_data_indices.size();
    c /= normalization;
    rho /= normalization;
    L /= normalization;

    // updated
    need_to_update_c_rho_L = false;
}

void TELQ::optimize_weights_Bellman_residual_error() {
    update_c_rho_L();
    // use small regularization to make solution unique
    L = L + 1e-10*eye(L.n_rows,L.n_cols);
    weights = solve((arma::mat)L,-1*rho);
    L = L - 1e-10*eye(L.n_rows,L.n_cols);
}

double TELQ::objective_and_gradient(col_vec_t& grad, const col_vec_t& weights) {
    update_c_rho_L();
    grad = 2*rho + 2*L*weights;
    return as_scalar(c + 2*rho.t()*weights + weights.t()*L*weights);
}

lbfgsfloatval_t TELQ::LBFGS_objective(const lbfgsfloatval_t* par, lbfgsfloatval_t* grad) {
    int nr_vars = weights.size();
    col_vec_t w(par,nr_vars);
    col_vec_t g(grad,nr_vars,false);
    double TD_error = objective_and_gradient(g,w);
    ++objective_evaluations;
    return TD_error;
}

int TELQ::LBFGS_progress(const lbfgsfloatval_t */*x*/,
                        const lbfgsfloatval_t */*g*/,
                        const lbfgsfloatval_t fx,
                        const lbfgsfloatval_t /*xnorm*/,
                        const lbfgsfloatval_t /*gnorm*/,
                        const lbfgsfloatval_t /*step*/,
                        int /*nr_variables*/,
                        int iteration_nr,
                        int /*ls*/) const {
    IF_DEBUG(1) { cout << "    Iteration " << iteration_nr << " (" << objective_evaluations << "), TD-error+L1 = " << fx << endl; }
    return 0;
}

void TELQ::LBFGS_final_message(double obj_val) const {
    IF_DEBUG(1) { cout << "    TD-error+L1 = " << obj_val << endl; }
}

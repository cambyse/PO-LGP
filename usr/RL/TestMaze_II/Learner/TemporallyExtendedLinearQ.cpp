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

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#define DEBUG_STRING "TEF: "
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

TELQ::row_vec_t TELQ::get_action_values(const_instance_ptr_t ins) const {
    int action_n = action_space->space_size();
    // return zero vector if value cannot be computed
    if(feature_set.size()==0) {
        DEBUG_OUT(1,"Cannot compute action (no features)");
        return zeros<row_vec_t>(action_n);
    }
    // get feature matrix
    f_mat_t feature_matrix = zeros<f_mat_t>(feature_set.size(),action_n);
    int feature_idx = 0;
    for(f_ptr_t feature : feature_set) {
        int action_idx = 0;
        for(action_ptr_t action : action_space) {
            feature_matrix(feature_idx,action_idx) = feature->evaluate(ins,action,observation_space,reward_space);
            ++action_idx;
        }
        ++feature_idx;
    }
    return weights.t()*feature_matrix;
}

// double TELQ::get_action_value(const_instance_ptr_t ins, action_ptr_t act) const {
//     // return zero if value cannot be computed
//     if(feature_set.size()==0) {
//         DEBUG_WARNING("Cannot compute action (no features)");
//         return 0;
//     }
//     // get feature values
//     row_vec_t feature_values(feature_set.size());
//     int feature_idx = 0;
//     for(f_ptr_t feature : feature_set) {
//         feature_values(feature_idx) = feature->evaluate(ins,act,observation_space,reward_space);
//         ++feature_idx;
//     }
//     // compute action value
//     return as_scalar(feature_values*weights);
// }

TELQ::action_ptr_t TELQ::get_action(const_instance_ptr_t ins) {
    row_vec_t action_values = get_action_values(ins);
    col_vec_t policy = util::soft_max((col_vec_t)action_values.t(), soft_max_temperature);
    int idx = util::draw_idx(policy);
    for(action_ptr_t action : action_space) {
        --idx;
        if(idx<0) {
            return action;
        }
    }
    DEBUG_DEAD_LINE;
    return action_space;
}

double TELQ::run_policy_iteration(bool final_L1) {
    int counter = 1;
    update_policy();
    double old_TD_error = get_TD_error();
    IF_DEBUG(1) { cout << "------------------------------BEGIN: Policy Iteration-----------------------------" << endl;}
    while(true) {
        // update weights
        optimize_weights_Bellman_residual_error();
        // get TD-error (and print message)
        double TD_error = get_TD_error();
        IF_DEBUG(1) {
            cout << "    Iteration: " << counter << ", TD-error: " << TD_error << " ( delta = " << old_TD_error-TD_error << ")" << endl;
        }
        // update policy (and break?)
        if(!update_policy() || old_TD_error<TD_error) {
            break;
        }
        // remember old TD-error
        old_TD_error = TD_error;
        ++counter;
    }
    // final L1-regularized optimization
    if(final_L1) {
        optimize_weights_LBFGS();
    }
    IF_DEBUG(1) { cout << "------------------------------END: Policy Iteration-------------------------------" << endl;}
    return get_TD_error();
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
            int action_idx = 0;
            for(action_ptr_t act : action_space) {
                cout << "    " << act << " --> " << get_action_values(ins)(action_idx) << endl;
                ++action_idx;
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

bool TELQ::update_policy() {

    // resize but remember old
    vector<col_vec_t> old_policy = policy;
    int action_n = action_space->space_size();


    // random policy if value function cannot be computed
    if(weights.size()==0) {
        col_vec_t random(action_n);
        random.fill(1./action_n);
        policy.assign(number_of_data_points,random);
    } else {

        // initialize to correct dimensions and set to zero
        policy.assign(number_of_data_points,zeros<col_vec_t>(action_n));

        // make sure all data are up-to-date
        update();

        // choose maximum value action efficiently using precomputed values
        for(int data_idx : Range(number_of_data_points)) {
            row_vec_t action_values = weights.t()*F_matrices[data_idx];
            policy[data_idx] = util::soft_max((col_vec_t)action_values.t(), soft_max_temperature);
        }

        // iterate through data
//#define FORCE_DEBUG_LEVEL 5
        // int data_idx = 0;
        // for(const_instance_ptr_t episode : instance_data) {
        //     for(const_instance_ptr_t ins_t=episode->const_first(); ins_t!=INVALID; ++ins_t) {
        //         row_vec_t action_values = weights.t()*F_matrices[data_idx];
        //         policy[data_idx] = util::soft_max((col_vec_t)action_values.t(), soft_max_temperature);
        //         IF_DEBUG(1) {
        //             bool finite = true;
        //             for(auto p : policy[data_idx]) {
        //                 if(!util::is_finite_number(p)) {
        //                     finite = false;
        //                 }
        //             }
        //             if(!finite) {
        //                 DEBUG_WARNING("Data idx (" << data_idx << ")");
        //                 policy[data_idx].t().print("policy");
        //                 action_values.print("values");
        //             }
        //         }
        //         IF_DEBUG(5) {
        //             cout << "(" << data_idx << ")" << ins_t << endl;
        //             cout << "            actions: ";
        //             for(auto a : action_space) {
        //                 cout << "	" << a;
        //             }
        //             cout << endl;
        //             cout << "       values (mat): ";
        //             action_values.print();
        //             cout << "    values (direct): ";
        //             get_action_values(ins_t).print();
        //             if(old_policy.size()==policy.size()) {
        //                 cout << "         old policy: ";
        //                 old_policy[data_idx].t().print();
        //             } else {
        //                 cout << "--- old policy not available ---" << endl;
        //             }
        //             cout << "         new policy: ";
        //             policy[data_idx].t().print();
        //             cout << endl;
        //         }
        //         ++data_idx;
        //     }
        // }
//#define FORCE_DEBUG_LEVEL 0

    }

    // set flag if changed
    if(policy_approx_equal(policy,old_policy)) {
        return false;
    } else {
//#define FORCE_DEBUG_LEVEL 5
        IF_DEBUG(5) {
            int data_idx = 0;
            for(const_instance_ptr_t episode : instance_data) {
                for(const_instance_ptr_t ins_t=episode->const_first(); ins_t!=INVALID; ++ins_t) {
                    if(!policy_approx_equal(old_policy[data_idx],policy[data_idx])) {
                        cout << "(" << data_idx << ") : " << ins_t << endl;
                        old_policy[data_idx].t().print("old policy");
                        (weights.t()*F_matrices[data_idx]).print("values");
                        policy[data_idx].t().print("new policy");
                    }
                    ++data_idx;
                }
            }
        }
//#define FORCE_DEBUG_LEVEL 0
        need_to_update_c_rho_L = true;
        return true;
    }
}

void TELQ::update_rewards_and_data_indices() {
    IF_DEBUG(1) {
        cout << "Update rewards and data-indices" << endl;
    }
    rewards_and_data_indices.clear();
    int data_idx = 0;
    for(const_instance_ptr_t episode : instance_data) {
        // first instance in each episode is skipped
        ++data_idx;
        for(const_instance_ptr_t ins_t=episode->const_first()->const_next(); ins_t!=INVALID; ++ins_t) {
            rewards_and_data_indices.push_back(tuple<double,int>(ins_t->reward->get_value(),data_idx));
            ++data_idx;
        }
        // take invalid episodes (zero or one instances) into account
        if(episode->const_first()->const_next()==INVALID) {
            --data_idx;
        }
    }
    assert(number_of_data_points=rewards_and_data_indices.size()+instance_data.size());
}

TELQ::col_vec_t TELQ::optimal_2x2_policy(const_instance_ptr_t ins) const {
    vector<action_ptr_t> optimal_actions;
    if(ins->observation==MazeObservation(2,2,0,0)) {
        optimal_actions.push_back(action_ptr_t(new MazeAction("d")));
        optimal_actions.push_back(action_ptr_t(new MazeAction("r")));
    } else if(ins->observation==MazeObservation(2,2,1,1)) {
        optimal_actions.push_back(action_ptr_t(new MazeAction("u")));
        optimal_actions.push_back(action_ptr_t(new MazeAction("l")));
    } else if(ins->observation==MazeObservation(2,2,0,1)) {
        if(ins->const_prev()->observation==MazeObservation(2,2,0,0)) {
            optimal_actions.push_back(action_ptr_t(new MazeAction("r")));
        } else {
            optimal_actions.push_back(action_ptr_t(new MazeAction("u")));
        }
    } else if(ins->observation==MazeObservation(2,2,1,0)) {
        if(ins->const_prev()->observation==MazeObservation(2,2,0,0)) {
            optimal_actions.push_back(action_ptr_t(new MazeAction("d")));
        } else {
            optimal_actions.push_back(action_ptr_t(new MazeAction("l")));
        }
    } else {
        DEBUG_ERROR("Unexpected observation unsing default action 'stay'");
        optimal_actions.push_back(action_ptr_t(new MazeAction("s")));
    }
    col_vec_t pol(action_space->space_size());
    pol.fill(0);
    int action_idx = 0;
    for(action_ptr_t a_elem : action_space) {
        for(action_ptr_t a_opt : optimal_actions) {
            if(a_elem==a_opt) {
                pol(action_idx) = 1./optimal_actions.size();
            }
        }
        ++action_idx;
    }
    return pol;
}

void TELQ::set_optimal_2x2_policy() {
    // set flag
    need_to_update_c_rho_L = true;
    // resize
    policy.resize(number_of_data_points);
    // set optimal policy
    int data_idx = 0;
    for(const_instance_ptr_t episode : instance_data) {
        for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {
            policy[data_idx] = optimal_2x2_policy(ins);
            ++data_idx;
        }
    }
}

void TELQ::update_c_rho_L() {
    // first update everything else
    update();

    // only update if necessary
    if(!need_to_update_c_rho_L) {
        return;
    }

    // update
    int feature_n = feature_set.size();
    c = 0;
    rho.zeros(feature_n);
    L.zeros(feature_n,feature_n);
    int progress_counter = 0;
    IF_DEBUG(1) { ProgressBar::init("Updating c-rho-L:          "); }

#ifdef USE_OMP
    vector<double> c_update;
    vector<col_vec_t> rho_update;
    vector<f_mat_t> L_update;
#pragma omp parallel
    {
        // compute updates in multiple threads
        int nr_threads = omp_get_num_threads();
#pragma omp critical
        {
            c_update.resize(nr_threads,0);
            rho_update.resize(nr_threads,zeros<col_vec_t>(rho.size()));
            L_update.resize(nr_threads,zeros<f_mat_t>(L.n_rows,L.n_cols));
        } // end OMP critical
#pragma omp barrier // wait so initialization is done before work starts

#pragma omp for
        for(unsigned int idx = 0; idx<rewards_and_data_indices.size(); ++idx) {

            // precompute all values
            tuple<double,int> reward_and_data_index = rewards_and_data_indices[idx];
            double r_t = get<0>(reward_and_data_index);
            int t_idx = get<1>(reward_and_data_index);
            col_vec_t phi = (col_vec_t)(discount * F_matrices[t_idx]*policy[t_idx] -
                                        F_matrices[t_idx-1].col(outcome_indices[t_idx]));

            // perform update
            int thread_nr = omp_get_thread_num();
            c_update[thread_nr] = c_update[thread_nr] + pow(r_t,2);
            rho_update[thread_nr] = rho_update[thread_nr] + r_t * phi;
            L_update[thread_nr] = L_update[thread_nr] + kron(phi,phi.t());
#pragma omp critical
            {
                ++progress_counter;
                IF_DEBUG(1) { ProgressBar::print(progress_counter,number_of_data_points); }
            } // end OMP critical

        } // end OMP for

    } // end OMP parallel

    // joint updates
    for(auto& c_ : c_update) {
        c = c + c_;
    }
    for(auto& rho_ : rho_update) {
        rho = rho + rho_;
    }
    for(auto& L_ : L_update) {
        L = L + L_;
    }

#else

    for(auto reward_and_idx : rewards_and_data_indices) {
        double r_t = get<0>(reward_and_idx);
        int t_idx = get<1>(reward_and_idx);
        //DEBUG_OUT(0,"T-idx: " << t_idx << " (" << policy_indices.size() << "/" << outcome_indices.size() << "/" << F_matrices.size() << ")");
        col_vec_t phi = (col_vec_t)(discount * F_matrices[t_idx].col(policy_indices[t_idx]) -
                                    F_matrices[t_idx-1].col(outcome_indices[t_idx]));
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

bool TELQ::policy_approx_equal(std::vector<col_vec_t> & p1, std::vector<col_vec_t> & p2) const {
    if(p1.size()!=p2.size()) {
        return false;
    } else {
        for(int idx=0; idx<(int)p1.size(); ++idx) {
            if(!policy_approx_equal(p1[idx],p2[idx])) return false;
        }
    }
    return true;
}

bool TELQ::policy_approx_equal(col_vec_t & p1, col_vec_t & p2) const {
    if(p1.size()!=p2.size()) {
        return false;
    } else {
        return arma::norm(p1-p2,"inf")<1e-3;
    }
}

lbfgsfloatval_t TELQ::LBFGS_objective(const lbfgsfloatval_t* par, lbfgsfloatval_t* grad) {
    int nr_vars = weights.size();
    col_vec_t w(par,nr_vars);
    col_vec_t g(grad,nr_vars,false);
    double TD_error = objective_and_gradient(g,w);
    ++objective_evaluations;
    return TD_error;
}

int TELQ::LBFGS_progress(const lbfgsfloatval_t * x,
                         const lbfgsfloatval_t */*g*/,
                         const lbfgsfloatval_t fx,
                         const lbfgsfloatval_t /*xnorm*/,
                         const lbfgsfloatval_t /*gnorm*/,
                         const lbfgsfloatval_t /*step*/,
                         int nr_variables,
                         int iteration_nr,
                         int /*ls*/) const {
    IF_DEBUG(1) {
        // L1 norm //
        double xnorm = 0;
        for(int idx=0; idx<nr_variables; ++idx) {
            xnorm += fabs(x[idx]);
        }
        cout <<
            QString("    Iteration %1 (%2), TD-error + L1 = %3 + %4")
            .arg(iteration_nr)
            .arg(objective_evaluations)
            .arg(fx-xnorm*l1_factor,11,'e',5)
            .arg(xnorm*l1_factor,11,'e',5)
             << endl;
    }
    return 0;
}

void TELQ::LBFGS_final_message(double obj_val) const {
    IF_DEBUG(1) {
        // L1 norm //
        double xnorm = arma::as_scalar(arma::sum(arma::abs(weights)));
        cout <<
            QString("    TD-error + L1 = %1 + %2")
            .arg(obj_val-xnorm*l1_factor,11,'e',5)
            .arg(xnorm*l1_factor,11,'e',5)
             << endl;
    }
}

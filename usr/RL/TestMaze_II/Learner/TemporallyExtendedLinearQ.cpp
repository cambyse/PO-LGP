#include "TemporallyExtendedLinearQ.h"

#include "ConjunctiveAdjacency.h"
#include "../util/util.h"
#include "../util/QtUtil.h"
#include "../util/ProgressBar.h"

#include <omp.h>
#define USE_OMP

#include <iomanip>

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using util::Range;
using util::INVALID;

using std::vector;
using std::map;
using std::make_tuple;
using std::dynamic_pointer_cast;

typedef TemporallyExtendedLinearQ TELQ;

TemporallyExtendedLinearQ::TemporallyExtendedLinearQ(std::shared_ptr<ConjunctiveAdjacency> N):
    TemporallyExtendedFeatureLearner(N)
{}

double TELQ::neg_log_likelihood(col_vec_t& grad, const col_vec_t& w) {

    /** Some issues with armadillo:
     *
     * -- simple exp(vector) prodoces 1x1 matrix, trunc_exp does not -- don't
     * know why */

#warning todo: write a minimal example for this behavior

    DEBUG_OUT(3,"Compute neg-log-likelihood");

    // check dimensions of input
    if(DEBUG_LEVEL>0) {
        if(grad.size()!=weights.size()){
            DEBUG_DEAD_LINE;
        }
        if(w.size()!=weights.size()) {
            DEBUG_DEAD_LINE;
        }
    }

    // initialize objective and gradient
    double obj = 0;
    grad.zeros(feature_set.size());

#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(1)
#endif

    // sum over data
    for(int data_idx=0; data_idx<(int)number_of_data_points; ++data_idx) {

        // get matrix and outcome index
        const int outcome_idx = outcome_indices[data_idx];
        const f_mat_t& F = F_matrices[data_idx];

        // debug output
        if(DEBUG_LEVEL>=3) {
            DEBUG_OUT(0,"Data point " << data_idx << ", outcome index " << outcome_idx);
            F.print("Sparse F-Matrix");
            w.print("Weights:");
        }

        // interim variables
        const row_vec_t lin = w.t()*F;
        const row_vec_t exp_lin = arma::trunc_exp(lin);
        const double z = sum(exp_lin);

        // debug output
        if(DEBUG_LEVEL>=3) {
            lin.print("lin:");
            exp_lin.print("exp_lin:");
        }

        // compute objective and gradient
        double obj_comp = lin(outcome_idx)-log(z);
        col_vec_t grad_comp = F.col(outcome_idx) - F*exp_lin.t()/z;

#ifdef USE_OMP
#pragma omp critical
#endif
        {
            // update
            obj += obj_comp;
            grad += grad_comp;
            if(DEBUG_LEVEL>=3) {
                grad_comp.print("gradient component:");
                DEBUG_OUT(0,"objective component " << obj_comp);
            }
        } // end critical
    } // end parallel

    // divide by number of data points and reverse sign
    if(number_of_data_points>0) {
        obj = -obj/number_of_data_points;
        grad = -grad/number_of_data_points;
    }

    if(DEBUG_LEVEL>=3) {
        DEBUG_OUT(0,"    Objective value: " << obj);
        DEBUG_OUT(0,"    Parameters	Gradient: ");
        for(int idx : Range(w.n_rows)) {
            DEBUG_OUT(0,"    " << w(idx) << "	" << grad(idx));
        }
    }

    // increment counter
    ++objective_evaluations;

    // return
    return obj;

}

lbfgsfloatval_t TELQ::LBFGS_objective(const lbfgsfloatval_t* par, lbfgsfloatval_t* grad) {
    int nr_vars = weights.size();
    col_vec_t w(par,nr_vars);
    col_vec_t g(grad,nr_vars,false);
    double neg_log_like = neg_log_likelihood(g,w);
    return neg_log_like;
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
    DEBUG_OUT(1,"Iteration " << iteration_nr << " (" << objective_evaluations << "), Likelihood = " << exp(-fx));
    //for(int x_idx : Range(nr_variables)) {
        //DEBUG_OUT(1, "    x[" << x_idx << "] = " << x[x_idx]);
    //}
    //DEBUG_OUT(1,"Iteration " << iteration_nr << " (Likelihood = " << exp(-fx) << ")" );
    return 0;
}


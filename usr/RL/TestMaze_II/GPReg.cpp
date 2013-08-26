#include "GPReg.h"

#include "util.h"

#define DEBUG_LEVEL 1
#include "debug.h"

using util::min;
using util::max;
using arma::eye;
using arma::as_scalar;

GPReg::GPReg(const double& g, const double& width, const double& mu, const double& sig):
    data_changed(false),
    gamma(g),
    k_width(width),
    prior_mu(mu),
    prior_sig(sig),
    lambda(sig*sig)
{}

GPReg::~GPReg() {}

void GPReg::add_new_point(const vec& x, const double& y) {
    x_data.insert_rows(x_data.n_rows,x);
    y_data.insert_rows(y_data.n_rows,vec(1).fill(y));
    data_changed = true;
}

void GPReg::add_new_point(const double& x, const double& y) {
    vec x_vec(1);
    x_vec.fill(x);
    add_new_point(x_vec,y);
}

double GPReg::get_mean(const vec& x) {
    update_data();
    vec k = querry_covariance_vector(x);
    mat tmp_K_inv_y = K_inv*y_data;
    mat tmp_ret = k.t()*tmp_K_inv_y;
    // x_data.print("x_data");
    // y_data.print("y_data");
    // k.print("k");
    // K_inv.print("K_inv");
    // tmp_K_inv_y.print("tmp_K_inv_y");
    // tmp_ret.print("tmp_ret");
    return as_scalar(tmp_ret)/x_data.n_rows;
}

double GPReg::get_mean(const double& x) {
    vec x_vec(1);
    x_vec.fill(x);
    return get_mean(x_vec);
}

void GPReg::update_data() {
    if(data_changed) {
        DEBUG_OUT(1,"Updating data...");
        // covariance matrix of data
        K = x_data*x_data.t();
        // inverse augmented covariance matrix
        K_inv = (K + lambda*eye(K.n_rows,K.n_cols)).i();
        // set flag
        data_changed = false;
        DEBUG_OUT(1,"    DONE");
    }
}

double GPReg::kernel(const vec& x1, const vec& x2) const {
    double k = 0;
    for(unsigned int i=0; i<min(x1.n_rows,x2.n_rows); ++i) {
        k += exp(-pow(fabs(x1[i]-x2[i])/k_width,gamma));
    }
    return k;
}

GPReg::vec GPReg::querry_covariance_vector(const vec& x) const {
    vec k(x_data.n_rows);
    for(unsigned int row_idx=0; row_idx<k.n_rows; ++row_idx) {
        k.row(row_idx) = kernel(x,x_data.row(row_idx));
    }
    return k;
}

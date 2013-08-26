#include "GPReg.h"

#include "util.h"

#define DEBUG_LEVEL 1;
#include "debug.h"

using util::min;
using util::max;

GPReg::GPReg(const double& g, const double& width, const double& mu, const double& sig):
    data_changed(false),
    gamma(g),
    prior_mu(mu),
    prior_sig(sig),
    lambda(sig*sig),
    k_width(width)
{}

GPReg::~GPReg() {}

void GPReg::add_new_point(const mat& x, const double& y) {
    x_data.append(x);
    y_data.append(y);
    data_changed = true;
}

double GPReg::get_mean(const mat& x) {
    return kernel_matrix(x,x_data)*K_inv*y_data;
}

void GPReg::update_data() {
    if(data_changed) {
        DEBUG_OUT(1,"Updating data...");
        K = x_data*x_data.t();
        K_inv = (K + lambda*arma::ident(K.dimx,K.dimy)).inv;
        data_changed = false;
        DEBUG_OUT(1,"    DONE");
    }
}

double GPReg::kernel(const vec& x1, const vec& x2) const {
    double k = 0;
    for(int i=0; i<min(x1.dimx,x2.dimx); ++i) {
        k += exp(-pow(fabs(x1[i]-x2[i])/k_width,gamma));
    }
    return k;
}

mat GPReg::kernel_matrix(const mat& x1, const mat& x2) const {
    mat k(x1.ydim,x2.xdim);
    for(int row_idx=0; row_idx<k.xdim; ++row_idx) {
        for(int col_idx=0; col_idx<k.ydim; ++col_idx) {
            k[row_idx][col_idx] = kernel(x1[][row_idx],x2[col_idx][]);
        }
    }
    return k;
}

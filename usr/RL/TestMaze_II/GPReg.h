#ifndef GPREG_H
#define GPREG_H

//#define ARMA_NO_DEBUG
#include <armadillo>

class GPReg {

public:

    typedef arma::mat mat;
    typedef arma::vec vec;

    GPReg(const double& g, const double& width, const double& mu, const double& sig);

    ~GPReg();

    void add_new_point(const mat& x, const double& y);

    double get_mean(const mat& x);

protected:
    bool data_changed;
    double gamma, k_width, prior_mu, prior_sig, lambda;
    mat x_data, K, K_inv;
    vec y_data;

    void update_data();
    double kernel(const vec& x1, const vec& x2) const;
    mat kernel_matrix(const mat& x1, const mat& x2) const;
};

#endif // GPREG_H

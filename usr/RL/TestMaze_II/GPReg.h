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

    void add_new_point(const vec& x, const double& y);
    void add_new_point(const double& x, const double& y); ///< Overload for 1D

    double get_mean(const vec& x);
    double get_mean(const double& x); ///< Overload for 1D

protected:
    bool data_changed;
    double gamma, k_width, prior_mu, prior_sig, lambda;
    mat x_data, K, K_inv;
    vec y_data;

    void update_data();
    double kernel(const vec& x1, const vec& x2) const;
    vec querry_covariance_vector(const vec& x) const;
};

#endif // GPREG_H

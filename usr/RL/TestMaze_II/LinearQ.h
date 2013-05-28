#ifndef LINEARQ_H_
#define LINEARQ_H_

#include "Data.h"
#include "Representation/Representation.h"
#include "Feature.h"

#include <vector>

class LinearQ
{
public:

    typedef Data::idx_t                      idx_t;
    typedef Data::size_t                     size_t;
    typedef Data::probability_t              probability_t;
    typedef Feature::feature_return_value    f_ret_t;
    typedef std::vector<const instance_t *>  instance_vector_t;

    LinearQ(const double&);
    virtual ~LinearQ();

    /** \brief Add a new instance to the tree. */
    void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward
    );

    /** \brief Optimize weights.
     *
     * Optimize the weights for the currently active features using a
     * \f$L^2\f$-regularization and returning the optimum value of the objective
     * function.
     *
     * @param reg Coefficient for the \f$L^2\f$-regularization. */
    double optimize(const double& reg);

    /** \brief Clears all data (all instances). */
    void clear_data();

    action_t get_max_value_action(const instance_t *);

    /*! \brief Set the discount rate used for computing state and action values. */
    void set_discount(const double& d) { discount = d; }

    /** \brief Constructs new candidate features. */
    void construct_candidate_features(const int& n);

private:

    //--------------//
    // General Data //
    //--------------//
    int k;                                           ///< Number of time steps in the past to be considered.
    instance_t * instance_data;                      ///< Data used for maximazing the likelihood.
    double discount;                                 ///< Discount for computing value.

    //------------------------//
    // Features, Weights etc. //
    //------------------------//
    std::vector<double> feature_weights;             ///< Coefficients for active features.
    std::vector<Feature*> basis_features;            ///< Basis features used to construct new candidates.
    std::vector<AndFeature> active_features;         ///< Set of currently active features.
    std::vector<AndFeature> candidate_features;      ///< Set of candidate features.

    //------------------//
    // Member Functions //
    //------------------//

    probability_t prior_probability(const state_t&, const reward_t&) const;
};

#endif /* LINEARQ_H_ */

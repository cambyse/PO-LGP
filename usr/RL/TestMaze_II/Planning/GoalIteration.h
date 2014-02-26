#ifndef GOALITERATION_H_
#define GOALITERATION_H_

#include "Policy.h"
#include "../Predictor.h"
#include "../SpaceManager.h"
#include "../Visualizer.h"

//#define ARMA_NO_DEBUG
#include <armadillo>

#include <map>

class GoalIteration: public Policy, public SpaceManager  {

public:

    typedef Visualizer::color_t color_t;
    typedef Visualizer::color_vector_t color_vector_t;

    DISAMBIGUATE_CONFIG_TYPEDEFS(Policy);

    GoalIteration(const double &, const Predictor &, bool auto_it = true);
    virtual ~GoalIteration() = default;

    virtual action_ptr_t get_action(const instance_t*);
    void set_goal(observation_ptr_t);
    void print_matrices() const;
    void iterate(const double & threshold = DBL_MAX);
    color_vector_t get_value_as_color() const;

private:

    std::map<action_ptr_t,std::map<observation_ptr_t,int> > a_o_idx_map;
    double discount;
    bool auto_iterate;
    arma::vec Q, R, V;
    arma::mat p;
};

#endif /* GOALITERATION_H_ */

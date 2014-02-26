#ifndef GOALITERATION_H_
#define GOALITERATION_H_

#include "Policy.h"
#include "../Predictor.h"
#include "../SpaceManager.h"

//#define ARMA_NO_DEBUG
#include <armadillo>

#include <map>

class GoalIteration: public Policy, public SpaceManager  {
public:
    DISAMBIGUATE_CONFIG_TYPEDEFS(Policy);
    GoalIteration(const double &, const Predictor &);
    virtual ~GoalIteration() = default;
    virtual action_ptr_t get_action(const instance_t*);
    void set_goal(observation_ptr_t);
    void print_matrices() const;
    void iterate();
private:
    std::map<action_ptr_t,std::map<observation_ptr_t,int> > a_o_idx_map;
    double discount;
    arma::vec Q, R, V;
    arma::mat p;
};

#endif /* GOALITERATION_H_ */

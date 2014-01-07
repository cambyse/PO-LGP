#ifndef FEATURELEARNER_H_
#define FEATURELEARNER_H_

#include "Config.h"
#include "Feature.h"

class Environment;

class FeatureLearner {

public:

    // typedefs/types
    USE_CONFIG_TYPEDEFS;
    typedef Feature::feature_return_value f_ret_t;
    typedef Feature::const_feature_ptr_t  f_ptr_t;
    enum class LEARNER_TYPE { FULL_PREDICTIVE, HISTORY_ONLY, HISTORY_AND_ACTION };

    // constructor/destructor
    FeatureLearner(const LEARNER_TYPE lt): learner_type(lt) {}
    virtual ~FeatureLearner() = default;

    /** \brief Initialize action, observation, and reward spaces. */
    virtual void set_spaces(const Environment & environment);

    /** \brief Set the spaces used for planning. */
    void set_spaces(const action_ptr_t & a, const observation_ptr_t & o, const reward_ptr_t & r);

    /** \brief Initialize the basis features used for learning. */
    virtual void set_features(const Environment & environment);

protected:

    const LEARNER_TYPE   learner_type;      ///< Characterize the different types of feature learners.
    action_ptr_t         action_space;      ///< The action space that is used.
    observation_ptr_t    observation_space; ///< The observation space that is used.
    reward_ptr_t         reward_space;      ///< The reward space that is used.
    std::vector<f_ptr_t> basis_features;    ///< Basis features used to construct new candidates.
};

#endif /* FEATURELEARNER_H_ */

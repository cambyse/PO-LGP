#ifndef FEATURELEARNER_H_
#define FEATURELEARNER_H_

#include "../Representation/Feature.h"
#include "../SpaceManager.h"

class Environment;

class FeatureLearner: public virtual SpaceManager {

public:

    // typedefs/types
    USE_CONFIG_TYPEDEFS;
    typedef Feature::feature_return_value f_ret_t;
    typedef Feature::const_feature_ptr_t  f_ptr_t;
    enum class LEARNER_TYPE { FULL_PREDICTIVE, HISTORY_ONLY, HISTORY_AND_ACTION };

    // constructor/destructor
    FeatureLearner(const LEARNER_TYPE lt): learner_type(lt) {}
    virtual ~FeatureLearner() = default;

    /** \brief Initialize the basis features used for learning. */
    virtual void set_features(const Environment & environment);

protected:

    const LEARNER_TYPE   learner_type;      ///< Characterize the different types of feature learners.
    std::vector<f_ptr_t> basis_features;    ///< Basis features used to construct new candidates.
};

#endif /* FEATURELEARNER_H_ */

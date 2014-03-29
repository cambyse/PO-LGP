//===================================================================

#ifndef X_SERVER
//#define X_SERVER
#endif

#ifndef BATCH_MODE
//#define BATCH_MODE
#endif

#ifndef BATCH_MODE_QUIET
//#define BATCH_MODE_QUIET
#endif

#ifndef NO_RANDOM
//#define NO_RANDOM
#endif

//===================================================================

#include "Representation/AbstractAction.h"
#include "Representation/AbstractObservation.h"
#include "Representation/AbstractReward.h"
#include "Representation/AbstractInstance.h"
#include "Representation/Feature.h"
#include "util/util.h"
#include <unordered_set>

//===================================================================

#ifndef CONFIG_TYPEDEFS_H_
#define CONFIG_TYPEDEFS_H_

#define USE_CONFIG_TYPEDEFS                                             \
    typedef unsigned long long int         large_size_t;                \
    typedef long long int                  idx_t;                       \
    typedef double                         probability_t;               \
    typedef AbstractAction::ptr_t          action_ptr_t;                \
    typedef AbstractObservation::ptr_t     observation_ptr_t;           \
    typedef AbstractReward::ptr_t          reward_ptr_t;                \
    typedef AbstractInstance::ptr_t        instance_ptr_t;              \
    typedef AbstractInstance::const_ptr_t  const_instance_ptr_t;        \
    typedef Feature::const_feature_ptr_t   f_ptr_t;                     \
    typedef Feature::feature_return_t      f_ret_t;                     \
    typedef std::unordered_set<f_ptr_t>    feature_set_t;

#define DISAMBIGUATE_CONFIG_TYPEDEFS(class)     \
    using class::large_size_t;                  \
    using class::idx_t;                         \
    using class::probability_t;                 \
    using class::action_ptr_t;                  \
    using class::observation_ptr_t;             \
    using class::reward_ptr_t;                  \
    using class::instance_ptr_t;                \
    using class::const_instance_ptr_t;          \
    using class::f_ptr_t;                       \
    using class::f_ret_t;                       \
    using class::feature_set_t;

#endif /* CONFIG_TYPEDEFS_H_ */

//===================================================================

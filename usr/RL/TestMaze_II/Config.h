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
#include <tuple>
#include <set>
#include <map>

//===================================================================

#ifndef CONFIG_TYPEDEFS_H_
#define CONFIG_TYPEDEFS_H_

#define BASIC_CONFIG_TYPEDEFS                                           \
    /** \brief Unsigned integer type for very large quantities. */      \
    typedef unsigned long long int         large_size_t;                \
    /** \brief Signed integer type for very large quantities. */        \
    typedef long long int                  idx_t;                       \
    /** \brief Type for representing probabilties. */                   \
    typedef double                         probability_t;

#define REPRESENTATION_CONFIG_TYPEDEFS                                  \
    /** \brief Type for (pointers to) abstract actions. */              \
    typedef AbstractAction::ptr_t          action_ptr_t;                \
    /** \brief Type for (pointers to) abstract observations. */         \
    typedef AbstractObservation::ptr_t     observation_ptr_t;           \
    /** \brief Type for (pointers to) abstract rewards. */              \
    typedef AbstractReward::ptr_t          reward_ptr_t;                \
    /** \brief Type for abstract instances, i.e., (sequences of)        \
     * (action,observation,reward)-triplets. */                         \
    typedef AbstractInstance::ptr_t        instance_ptr_t;              \
    /** \brief Constant version of instance_ptr_t. */                   \
    typedef AbstractInstance::const_ptr_t  const_instance_ptr_t;        \
    /** \brief Map from observation-reward pairs to probabilies. */     \
    typedef std::map<std::tuple<observation_ptr_t,reward_ptr_t>,probability_t> probability_map_t;

#define FEATURE_CONFIG_TYPEDEFS                                         \
    /** \brief Type for (pointers to) abstract features. */             \
    typedef Feature::const_feature_ptr_t   f_ptr_t;                     \
    /** \brief Return type of features. */                              \
    typedef Feature::feature_return_t      f_ret_t;                     \
    /** \brief Set with (pointers to) unique features (objects are      \
     * compared).  */                                                   \
    typedef std::set<f_ptr_t, util::deref_less<f_ptr_t> > f_set_t;      \
    /** \brief Set with unique (pointers to) features (adresses are     \
     * compared).  */                                                   \
    typedef std::unordered_set<f_ptr_t>    f_ptr_set_t;                 \


#define USE_CONFIG_TYPEDEFS                     \
    BASIC_CONFIG_TYPEDEFS                       \
    REPRESENTATION_CONFIG_TYPEDEFS              \
    FEATURE_CONFIG_TYPEDEFS                     \

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
    using class::f_ptr_set_t;                   \
    using class::f_set_t;                       \
    using class::probability_map_t

#endif /* CONFIG_TYPEDEFS_H_ */

//===================================================================

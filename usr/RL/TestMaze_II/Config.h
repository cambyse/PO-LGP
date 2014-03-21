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

#ifndef CONFIG_TYPE_INCLUDES_H_
#define CONFIG_TYPE_INCLUDES_H_

#include "Representation/AbstractAction.h"
#include "Representation/AbstractObservation.h"
#include "Representation/AbstractReward.h"
#include "Representation/AbstractInstance.h"

#endif /* CONFIG_TYPE_INCLUDES_H_ */

//===================================================================

#ifndef CONFIG_TYPEDEFS_H_
#define CONFIG_TYPEDEFS_H_

#define USE_CONFIG_TYPEDEFS                                     \
    typedef unsigned long long int     size_t;                  \
    typedef long long int              idx_t;                   \
    typedef double                     probability_t;           \
    typedef AbstractAction::ptr_t      action_ptr_t;            \
    typedef AbstractObservation::ptr_t observation_ptr_t;       \
    typedef AbstractReward::ptr_t      reward_ptr_t;            \
    typedef AbstractInstance::ptr_t    instance_ptr_t;          \
    typedef AbstractInstance::const_ptr_t const_instance_ptr_t;

#define DISAMBIGUATE_CONFIG_TYPEDEFS(class)     \
    using class::size_t;                        \
    using class::idx_t;                         \
    using class::probability_t;                 \
    using class::action_ptr_t;                  \
    using class::observation_ptr_t;             \
    using class::reward_ptr_t;                  \
    using class::instance_ptr_t;                \
    using class::const_instance_ptr_t;

#endif /* CONFIG_TYPEDEFS_H_ */

//===================================================================

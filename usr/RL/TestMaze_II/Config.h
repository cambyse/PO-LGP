//===================================================================

#ifndef X_SERVER
//#define X_SERVER
#endif

#ifndef BATCH_MODE
#define BATCH_MODE
#endif

#ifndef BATCH_MODE_QUIET
#define BATCH_MODE_QUIET
#endif

#ifndef NO_RANDOM
//#define NO_RANDOM
#endif

//===================================================================

#ifndef CONFIG_TYPEDEFS_H_
#define CONFIG_TYPEDEFS_H_

#define USE_CONFIG_TYPEDEFS                             \
    typedef unsigned long long int size_t;              \
    typedef long long int          idx_t;               \
    typedef double                 probability_t;       \
    typedef Action                 action_t;            \
    typedef ActionIt               actionIt_t;          \
    typedef AugmentedObservation   observation_t;       \
    typedef AugmentedObservationIt observationIt_t;     \
    typedef EnumeratedReward       reward_t;            \
    typedef EnumeratedRewardIt     rewardIt_t;          \
    typedef Instance               instance_t;          \
    typedef InstanceIt             instanceIt_t;        \
    typedef ConstInstanceIt        const_instanceIt_t;

#endif /* CONFIG_TYPEDEFS_H_ */

//===================================================================

#ifndef CONFIG_TYPE_INCLUDES_H_
#define CONFIG_TYPE_INCLUDES_H_

#define USE_AUGMENTED_OBSERVATION

#include "Representation/Action.h"

#ifdef USE_AUGMENTED_OBSERVATION
#include "Representation/AugmentedObservation.h"
#else
#include "Representation/Observation.h"
#endif

#include "Representation/EnumeratedReward.h"

#include "Representation/Instance.h"

#endif /* CONFIG_TYPE_INCLUDES_H_ */

//===================================================================

#ifndef CONFIG_H_
#define CONFIG_H_

class Config {
public:
    // typedefs
    typedef unsigned long long int   size_t;
    // constants
    static const size_t    maze_x_size;
    static const size_t    maze_y_size;
    static const size_t    k;
};

#endif /* CONFIG_H_ */

//===================================================================




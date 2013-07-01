#include "Config.h"
#include "Action.h"
#include "State.h"
#include "Instance.h"

#define USE_REPRESENTATION_TYPEDEFS                     \
    typedef Action              action_t;               \
    typedef ActionIt            actionIt_t;             \
    typedef State               state_t;                \
    typedef StateIt             stateIt_t;              \
    typedef Config::reward_t    reward_t;               \
    typedef Config::rewardIt_t  rewardIt_t;             \
    typedef Instance            instance_t;             \
    typedef InstanceIt          instanceIt_t;           \
    typedef ConstInstanceIt     const_instanceIt_t;

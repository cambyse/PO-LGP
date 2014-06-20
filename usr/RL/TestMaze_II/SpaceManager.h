#ifndef SPACEMANAGER_H_
#define SPACEMANAGER_H_

#include "Config.h"

class SpaceManager {
public:
    USE_CONFIG_TYPEDEFS;
    SpaceManager() = default;
    virtual ~SpaceManager() = default;
    virtual void get_spaces(action_ptr_t & a,
                            observation_ptr_t & o,
                            reward_ptr_t & r) const;
    virtual void set_spaces(const action_ptr_t & a,
                            const observation_ptr_t & o,
                            const reward_ptr_t & r);
    virtual void adopt_spaces(const SpaceManager & s) final;
protected:
    action_ptr_t action_space;           ///< Action space.
    observation_ptr_t observation_space; ///< State space.
    reward_ptr_t reward_space;           ///< Reward space.
};

#endif /* SPACEMANAGER_H_ */

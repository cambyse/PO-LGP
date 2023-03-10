#ifndef POLICY_H_
#define POLICY_H_

#include <config/Config.h>

class Policy {

public:

    USE_CONFIG_TYPEDEFS;

    Policy() = default;

    virtual ~Policy() = default;

    virtual action_ptr_t get_action(const_instance_ptr_t) = 0;

};

#endif /* POLICY_H_ */

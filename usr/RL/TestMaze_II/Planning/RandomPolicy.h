#ifndef RANDOMPOLICY_H_
#define RANDOMPOLICY_H_

#include "Policy.h"

class RandomPolicy: public Policy {
public:
    RandomPolicy(action_ptr_t a) { action_space = a; }
    virtual ~RandomPolicy() = default;
    virtual action_ptr_t get_action(const_instance_ptr_t) { return action_space->random_element(); }
private:
    action_ptr_t action_space;
};

#endif /* RANDOMPOLICY_H_ */

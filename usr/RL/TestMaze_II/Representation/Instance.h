/** @file This file implements the Instance classe. */

#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "Action.h"
#include "State.h"
#include "Reward.h"

#include <vector>

/** \brief Instance object */
class Instance {
protected:
    typedef std::vector<Instance*> container_t;
public:
    Action action;
    State state;
    Reward reward;
    Instance();
    Instance & operator++();
    Instance & operator--();
    bool is_valid() const;
protected:
    Instance * previous_instance, * next_instance;
    container_t * container;
};

#endif // INSTANCE_H_

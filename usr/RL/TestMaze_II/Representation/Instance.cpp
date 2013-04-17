#include "Instance.h"

using util::INVALID;

Instance::Instance(
    const Action& a,
    const State& s,
    const Reward& r,
    Instance * prev,
    Instance * next
    ):
    util::InvalidAdapter<Instance>(false),
    action(a),
    state(s),
    reward(r),
    previous_instance(prev),
    next_instance(next)
{}

Instance::~Instance() {
    // delete instance and also fix previous and next
}

Instance & Instance::operator++() {
    if(next_instance==nullptr) {
        this->invalidate();
    } else {
        (*this) = (*next_instance);
    }
    return *this;
}

Instance & Instance::operator--() {
    if(previous_instance==nullptr) {
        this->invalidate();
    } else {
        (*this) = (*previous_instance);
    }
    return *this;
}

Instance & Instance::insert_instance_after(
    const Action& a,
    const State& s,
    const Reward& r
    ) {
    Instance * new_instance = new Instance(a,s,r);

    new_instance->next_instance = this->next_instance;
    if(this->next_instance!=nullptr) {
        this->next_instance->previous_instance = new_instance;
    }

    new_instance->previous_instance = this;
    this->next_instance = new_instance;

    return *new_instance;
}

Instance & Instance::insert_instance_before(
    const Action& a,
    const State& s,
    const Reward& r
    ) {
    Instance * new_instance = new Instance(a,s,r);

    new_instance->previous_instance = this->previous_instance;
    if(this->previous_instance!=nullptr) {
        this->previous_instance->next_instance = new_instance;
    }

    new_instance->next_instance = this;
    this->previous_instance = new_instance;

    return *new_instance;
}

Instance & Instance::append_instance(
    const Action& a,
    const State& s,
    const Reward& r
    ) {
    Instance * current_instance = this;
    while(current_instance->next_instance!=nullptr) {
        current_instance = current_instance->next_instance;
    }
    return current_instance->insert_instance_after(a,s,r);
}

Instance & Instance::prepend_instance(
    const Action& a,
    const State& s,
    const Reward& r
    ) {
    Instance * current_instance = this;
    while(current_instance->previous_instance!=nullptr) {
        current_instance = current_instance->previous_instance;
    }
    return current_instance->insert_instance_before(a,s,r);
}

Instance Instance::first() const {
    const Instance * current_instance = this;
    while(current_instance->previous_instance!=nullptr) {
        current_instance = current_instance->previous_instance;
    }
    return *current_instance;
}

Instance Instance::last() const {
    const Instance * current_instance = this;
    while(current_instance->next_instance!=nullptr) {
        current_instance = current_instance->next_instance;
    }
    return *current_instance;
}

std::ostream& operator<<(std::ostream &out, const Instance& i) {
    out << "(" <<
        i.action << ", " <<
        i.state << ", " <<
        i.reward << ")";
    return out;
}

InstanceIt::InstanceIt() {
    *this = first();
}

InstanceIt::InstanceIt( const ActionIt& a, const StateIt& s, const RewardIt& r ):
    util::InvalidAdapter<InstanceIt>(false),
    action(a), state(s), reward(r)
{}

InstanceIt & InstanceIt::operator++() {
    ++reward;
    if(reward==INVALID) {
        reward = RewardIt::first();
        ++state;
        if(state==INVALID) {
            state = StateIt::first();
            ++action;
            if(action==INVALID) {
                action = ActionIt::first();
                this->invalidate();
            }
        }
    }
    return *this;
}

InstanceIt & InstanceIt::operator--() {
    --reward;
    if(reward==INVALID) {
        reward = RewardIt::last();
        --state;
        if(state==INVALID) {
            state = StateIt::last();
            --action;
            if(action==INVALID) {
                action = ActionIt::last();
                this->invalidate();
            }
        }
    }
    return *this;
}

const InstanceIt InstanceIt::first() {
    return InstanceIt(ActionIt::first(), StateIt::first(), RewardIt::first());
}

const InstanceIt InstanceIt::last() {
    return InstanceIt(ActionIt::last(), StateIt::last(), RewardIt::last());
}

std::ostream& operator<<(std::ostream &out, const InstanceIt& i) {
    out << (Instance)i;
    return out;
}

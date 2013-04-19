#include "Instance.h"

using util::INVALID;

static Instance * create(const Instance& i) {
    return new Instance(i);
}

static Instance * create(
    const Action& a = Action(),
    const State& s = State(),
    const Reward& r = Reward(),
    const Instance * prev = nullptr,
    const Instance * next = nullptr
    ) {
    return new Instance(a,s,r,prev,next);
}

Instance::~Instance() {
    // Call destructor of next and previous instance if they are in a proper chain.
    if(next_instance!=nullptr && next_instance->previous_instance==this) {
        next_instance->previous_instance=nullptr;
        delete next_instance;
        next_instance=nullptr;
    }
    if(previous_instance!=nullptr && previous_instance->next_instance==this) {
        previous_instance->next_instance=nullptr;
        delete previous_instance;
        previous_instance=nullptr;
    }
}

bool Instance::operator<(const Instance& other) const {
    if(action<other.action) return true;
    else if(action>other.action) return false;
    else if(state<other.state) return true;
    else if(state>other.state) return false;
    else if(reward<other.reward) return true;
    else if(reward>other.reward) return false;
    else return false;
}

Instance * Instance::insert_instance_after(
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

    return new_instance;
}

Instance * Instance::insert_instance_before(
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

    return new_instance;
}

Instance * Instance::append_instance(
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

Instance * Instance::prepend_instance(
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

InstanceIt Instance::it() const {
    return InstanceIt(this);
}

InstanceIt Instance::first() const {
    const Instance * current_instance = this;
    while(current_instance->previous_instance!=nullptr) {
        current_instance = current_instance->previous_instance;
    }
    return InstanceIt(current_instance);
}

InstanceIt Instance::last() const {
    const InstanceIt * current_instance = this;
    while(current_instance->next_instance!=nullptr) {
        current_instance = current_instance->next_instance;
    }
    return InstanceIt(current_instance);
}

std::ostream& operator<<(std::ostream &out, const Instance& i) {
    out << "(" <<
        i.action << ", " <<
        i.state << ", " <<
        i.reward << ")";
    return out;
}

Instance::Instance(
    const Action& a,
    const State& s,
    const Reward& r,
    const Instance * prev,
    const Instance * next
    ):
    action(a),
    state(s),
    reward(r),
    previous_instance(prev),
    next_instance(next)
{}

InstanceIt::InstanceIt():
    util::InvalidAdapter<InstanceIt>(true),
    this_instance(nullptr)
{}

InstanceIt::InstanceIt(const Instance * i):
    util::InvalidAdapter<InstanceIt>(false),
    this_instance(i)
{}

InstanceIt::operator Instance() const {
    return Instance(*this_instance);
}

const Instance * InstanceIt::operator->() {
    return this_instance;
}

InstanceIt & InstanceIt::operator++() {
    if(this_instance==nullptr || this_instance->next_instance==nullptr) {
        this->invalidate();
    } else {
        this_instance = this_instance->next_instance;
    }
    return *this;
}

InstanceIt & InstanceIt::operator--() {
    if(this_instance==nullptr || this_instance->previous_instance==nullptr) {
        this->invalidate();
    } else {
        this_instance = this_instance->previous_instance;
    }
    return *this;
}

InstanceIt & InstanceIt::operator+=(const int& c) {
    if(c<0) {
        return (*this) -= -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            ++(*this);
        }
        return (*this);
    }
}

InstanceIt & InstanceIt::operator-=(const int& c) {
    if(c<0) {
        return (*this) += -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            --(*this);
        }
        return (*this);
    }
}

InstanceIt operator+(const int& c) {
    if(c<0) {
        return (*this)-(-c);
    } else {
        InstanceIt ret = *this;
        for(int i=0; i<c && ret!=INVALID; ++i, ++ret) {}
        return ret;
    }
}

InstanceIt operator-(const int& c) {
    if(c<0) {
        return (*this)+(-c);
    } else {
        InstanceIt ret = *this;
        for(int i=0; i<c && ret!=INVALID; ++i, --ret) {}
        return ret;
    }
}

unsigned int InstanceIt::length_to_first() const {
    unsigned int counter = 0;
    for(InstanceIt ins=(*this)-1; ins!=INVALID; --ins) {
        ++counter;
    }
    return counter;
}

unsigned int InstanceIt::length_to_last() const {
    unsigned int counter = 0;
    for(InstanceIt ins=(*this)+1; ins!=INVALID; ++ins) {
        ++counter;
    }
    return counter;
}

// InstanceIt::InstanceIt() {}

// InstanceIt::InstanceIt( const ActionIt& a, const StateIt& s, const RewardIt& r ):
//     util::InvalidAdapter<InstanceIt>(false),
//     action(a), state(s), reward(r)
// {}

// InstanceIt & InstanceIt::operator++() {
//     ++reward;
//     if(reward==INVALID) {
//         reward = RewardIt::first();
//         ++state;
//         if(state==INVALID) {
//             state = StateIt::first();
//             ++action;
//             if(action==INVALID) {
//                 action = ActionIt::first();
//                 this->invalidate();
//             }
//         }
//     }
//     return *this;
// }

// InstanceIt & InstanceIt::operator--() {
//     --reward;
//     if(reward==INVALID) {
//         reward = RewardIt::last();
//         --state;
//         if(state==INVALID) {
//             state = StateIt::last();
//             --action;
//             if(action==INVALID) {
//                 action = ActionIt::last();
//                 this->invalidate();
//             }
//         }
//     }
//     return *this;
// }

// InstanceIt & InstanceIt::operator+=(const int& c) {
//     if(c<0) {
//         return (*this) -= -c;
//     } else {
//         for(int i=0; i<c && (*this)!=INVALID; ++i) {
//             ++(*this);
//         }
//         return (*this);
//     }
// }

// InstanceIt & InstanceIt::operator-=(const int& c) {
//     if(c<0) {
//         return (*this) += -c;
//     } else {
//         for(int i=0; i<c && (*this)!=INVALID; ++i) {
//             --(*this);
//         }
//         return (*this);
//     }
// }

// const InstanceIt InstanceIt::first() {
//     return InstanceIt(ActionIt::first(), StateIt::first(), RewardIt::first());
// }

// const InstanceIt InstanceIt::last() {
//     return InstanceIt(ActionIt::last(), StateIt::last(), RewardIt::last());
// }

// std::ostream& operator<<(std::ostream &out, const InstanceIt& i) {
//     out << (Instance)i;
//     return out;
// }

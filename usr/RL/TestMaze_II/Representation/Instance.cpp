#include "Instance.h"

using util::INVALID;

Instance * Instance::create(const Instance& i) {
    return new Instance(i);
}

Instance * Instance::create(
    const Action& a,
    const State& s,
    const Reward& r,
    const Instance * prev,
    const Instance * next
    ) {
    Instance * ins = new Instance(a,s,r);
    ins->previous_const = true;
    ins->next_const = true;
    ins->const_previous_instance = prev;
    ins->const_next_instance = next;
    return ins;
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

    // create new instance
    Instance * new_instance = new Instance(*this);
    new_instance->action = a;
    new_instance->state = s;
    new_instance->reward = r;

    // make next instance point back to newly created (if not const)
    if(this->next_instance!=nullptr) {
        this->next_instance->previous_instance = new_instance;
    }

    // make new instance point back to this instance (adjust const settings)
    new_instance->previous_const = false;
    new_instance->const_previous_instance = nullptr;
    new_instance->previous_instance = this;

    // make this instance point forward to new instance (adjust const settings)
    this->next_const = false;
    this->const_next_instance = nullptr;
    this->next_instance = new_instance;

    return new_instance;
}

Instance * Instance::insert_instance_before(
    const Action& a,
    const State& s,
    const Reward& r
    ) {

    // create new instance
    Instance * new_instance = new Instance(*this);
    new_instance->action = a;
    new_instance->state = s;
    new_instance->reward = r;

    // make previous instance point forward to newly created (if not const)
    if(this->previous_instance!=nullptr) {
        this->previous_instance->next_instance = new_instance;
    }

    // make new instance point forward to this instance (adjust const settings)
    new_instance->next_const = false;
    new_instance->const_next_instance = nullptr;
    new_instance->next_instance = this;

    // make this instance point back to new instance (adjust const settings)
    this->previous_const = false;
    this->const_previous_instance = nullptr;
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

const Instance * Instance::get_previous() const {
    return (previous_const ? const_previous_instance : previous_instance);
}

const Instance * Instance::get_next() const {
    return (next_const ? const_next_instance : next_instance);
}

InstanceIt Instance::it() const {
    return InstanceIt(this);
}

InstanceIt Instance::first() const {
    const Instance * current_instance = this;
    while(current_instance->get_previous()!=nullptr) {
        current_instance = current_instance->get_previous();
    }
    return InstanceIt(current_instance);
}

InstanceIt Instance::last() const {
    const Instance * current_instance = this;
    while(current_instance->get_next()!=nullptr) {
        current_instance = current_instance->get_next();
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
    const Reward& r
    ):
    action(a),
    state(s),
    reward(r),
    previous_instance(nullptr),
    next_instance(nullptr),
    const_previous_instance(nullptr),
    const_next_instance(nullptr),
    previous_const(false),
    next_const(false)
{}

InstanceIt::InstanceIt():
    util::InvalidAdapter<InstanceIt>(true),
    this_instance(nullptr)
{}

InstanceIt::InstanceIt(const Instance * i):
    util::InvalidAdapter<InstanceIt>(false),
    this_instance(i)
{}

InstanceIt::operator Instance*() const {
    return this_instance;
}

const Instance * InstanceIt::operator->() {
    return this_instance;
}

InstanceIt & InstanceIt::operator++() {
    if(this_instance==nullptr || this_instance->get_next()==nullptr) {
        this->invalidate();
    } else {
        this_instance = this_instance->get_next();
    }
    return *this;
}

InstanceIt & InstanceIt::operator--() {
    if(this_instance==nullptr || this_instance->get_previous()==nullptr) {
        this->invalidate();
    } else {
        this_instance = this_instance->get_previous();
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

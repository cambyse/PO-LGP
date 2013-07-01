#include "Instance.h"

#include <QTime>

#define DEBUG_STRING "Instance: "
#define DEBUG_LEVEL 1
#include "../debug.h"

using util::INVALID;
using std::cout;
using std::endl;

Instance::All::All(Instance * i): instance(i) {}
InstanceIt Instance::All::begin() { return instance->first(); }
InstanceIt Instance::All::end() { return InstanceIt(); }
Instance::ConstAll::ConstAll(const Instance * i): instance(i) {}
ConstInstanceIt Instance::ConstAll::begin() { return instance->const_first(); }
ConstInstanceIt Instance::ConstAll::end() { return ConstInstanceIt(); }

Instance * Instance::create(
    const action_t& a,
    const state_t& s,
    const reward_t& r,
    const Instance * prev,
    const Instance * next
    ) {
    Instance * ins = new Instance(a,s,r);
    ins->const_previous_instance = prev;
    ins->const_next_instance = next;
    return ins;
}

// Instance * Instance::create(const Instance& i) {
//     Instance * ins = new Instance(i);
//     return ins;
// }

Instance & Instance::operator=(const Instance& other) {
    action = other.action;
    state = other.state;
    reward = other.reward;
    all = All(this);
    const_all =  ConstAll(this);
    previous_instance = other.previous_instance;
    next_instance = other.next_instance;
    const_previous_instance = other.const_previous_instance;
    const_next_instance = other.const_next_instance;
    container = other.container;
    container_idx = other.container_idx;
    return *this;
}

Instance::~Instance() {

    container_t garbage;
    Instance *current_instance, *next, *previous;

    // collect forward
    current_instance = this->next_instance;
    while(current_instance!=nullptr) {
        next = current_instance->next_instance;
        garbage.push_back(current_instance);
        current_instance->next_instance = nullptr;
        current_instance->previous_instance = nullptr;
        if(current_instance->container!=this->container) {
            DEBUG_OUT(0,"Error: Instance to delete has different container assigned, which will not be deleted.");
        }
        current_instance->container = nullptr;
        current_instance=next;
    }

    // collect backwards
    current_instance = this->previous_instance;
    while(current_instance!=nullptr) {
        previous = current_instance->previous_instance;
        garbage.push_back(current_instance);
        current_instance->next_instance = nullptr;
        current_instance->previous_instance = nullptr;
        if(current_instance->container!=this->container) {
            DEBUG_OUT(0,"Error: Instance to delete has different container assigned, which will not be deleted.");
        }
        current_instance->container = nullptr;
        current_instance=previous;
    }

    // call destructors
    for(unsigned int idx=0; idx<garbage.size(); ++idx) {
        delete garbage[idx];
    }

    delete container;
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
    const action_t& a,
    const state_t& s,
    const reward_t& r
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
    new_instance->const_previous_instance = nullptr;
    new_instance->previous_instance = this;

    // make this instance point forward to new instance (adjust const settings)
    this->const_next_instance = nullptr;
    this->next_instance = new_instance;

    // adjust container
    if(this->container!=nullptr) {
        fill_container(this);
    }

    return new_instance;
}

Instance * Instance::insert_instance_before(
    const action_t& a,
    const state_t& s,
    const reward_t& r
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
    new_instance->const_next_instance = nullptr;
    new_instance->next_instance = this;

    // make this instance point back to new instance (adjust const settings)
    this->const_previous_instance = nullptr;
    this->previous_instance = new_instance;

    // adjust container
    if(this->container!=nullptr) {
        fill_container(this->previous_instance);
    }

    return new_instance;
}

Instance * Instance::append_instance(
    const action_t& a,
    const state_t& s,
    const reward_t& r
    ) {
    return this->last()->insert_instance_after(a,s,r);
}

Instance * Instance::prepend_instance(
    const action_t& a,
    const state_t& s,
    const reward_t& r
    ) {
    return this->first()->insert_instance_before(a,s,r);
}

InstanceIt Instance::it() {
    return InstanceIt(this);
}

ConstInstanceIt Instance::const_it() const {
    return ConstInstanceIt(this);
}

InstanceIt Instance::first() {
    Instance * current_instance;
    if(this->container!=nullptr) {
        current_instance = this->container->front();
    } else {
        current_instance = this;
        while(current_instance->get_non_const_previous()!=nullptr) {
            current_instance = current_instance->get_non_const_previous();
        }
    }
    return current_instance->it();
}

InstanceIt Instance::last() {
    Instance * current_instance;
    if(this->container!=nullptr) {
        current_instance = this->container->back();
    } else {
        current_instance = this;
        while(current_instance->get_non_const_next()!=nullptr) {
            current_instance = current_instance->get_non_const_next();
        }
    }
    return current_instance->it();
}

ConstInstanceIt Instance::const_first() const {
    const Instance * current_instance = this;
    while(current_instance->get_previous()!=nullptr) {
        current_instance = current_instance->get_previous();
    }
    return current_instance->const_it();
}

ConstInstanceIt Instance::const_last() const {
    const Instance * current_instance = this;
    while(current_instance->get_next()!=nullptr) {
        current_instance = current_instance->get_next();
    }
    return current_instance->const_it();
}

void Instance::set_container() {
    if(container!=nullptr) {
        DEBUG_OUT(1,"Container already set");
    } else {
        Instance * first_ins = this->first(); // remember before setting container
        first_ins->container = new container_t(1,this);
        first_ins->container_idx = 0;
        fill_container(first_ins);
    }
    return;
}

void Instance::unset_container() {
    if(container==nullptr) {
        DEBUG_OUT(1,"Container not set");
    } else {
        container_t * tmp_container = container; // remember
        unset_container_elements();
        if(container!=nullptr || container_idx!=-1) {
            DEBUG_OUT(0,"Error: Container was not correctly unset (ptr=" << container << ", idx=" << container_idx << ")");
        }
        delete tmp_container;
    }
    return;
}

std::ostream& operator<<(std::ostream &out, const Instance& i) {
    out << "(" <<
        i.action << ", " <<
        i.state << ", " <<
        i.reward << ")";
    return out;
}

const char* Instance::print() PRINT_FROM_OSTREAM;

// void Instance::set_previous(const Instance * prev) {
//     if(const_previous_instance!=nullptr) {
//         const_previous_instance=prev;
//     } else if(previous_instance!=nullptr) {
//         if(container!=nullptr) {
//             Instance * old_previous = previous_instance;
//             this->unset_container();
//             previous_instance = prev;
//             old_previous->set_container();
//             this->set_container();
//         } else {
//             previous_instance = prev;
//         }
//     }
// }

// void Instance::set_next(const Instance * next) {
//     if(const_next_instance!=nullptr) {
//         const_next_instance=next;
//     } else if(next_instance!=nullptr) {
//         if(container!=nullptr) {
//             Instance * old_next = next_instance;
//             this->unset_container();
//             next_instance = next;
//             old_next->set_container();
//             this->set_container();
//         } else {
//             next_instance = next;
//         }
//     }
// }

void Instance::check_performance_and_memory(bool memory) {

    //====================================//
    //     Destructor and Memory Leaks    //
    //====================================//

    if(memory) {
        int del_counter = 1;
        while(true) {
            Instance * in = Instance::create(action_t::STAY, 0, 0);
            for(int i=1; i<=1000000; ++i) {
                in = in->append_instance(action_t::STAY, i, 0);

            }
            in->set_container();
            delete in;
            cout << del_counter++ << endl;
        }
        return;
    }

    QTime timer;

    //=======================//
    //     Initialization    //
    //=======================//

    int items_to_create = 5;
    int min_ms = 2000;
    cout << "# Performing " << items_to_create << " initializations of at least " << min_ms << " msec" << endl;
    cout << "# items	time/ms" << endl;

    // without assignment (no container)
    cout << endl << endl << "# without assignment (no container)" << endl;
    for(int create_counter=0 ; create_counter<items_to_create; ++create_counter) {
        timer.restart();
        Instance * ins = Instance::create(action_t::STAY, 0, 0);
        int counter = 1;
        int time = timer.elapsed();
        cout << 0 << "	" << 0 << endl;
        while(timer.elapsed()<min_ms) {
            ins->append_instance(action_t::STAY, 0, 0);
            if(timer.elapsed()!=time && timer.elapsed()%100==0) {
                time = timer.elapsed();
                cout << time << "	" << counter << endl;
            }
            ++counter;
        }
        delete ins;
    }

    // with assignment (no container)
    cout << endl << endl << "# with assignment (no container)" << endl;
    for(int create_counter=0 ; create_counter<items_to_create; ++create_counter) {
        timer.restart();
        Instance * ins = Instance::create(action_t::STAY, 0, 0);
        int counter = 1;
        int time = timer.elapsed();
        cout << 0 << "	" << 0 << endl;
        while(timer.elapsed()<min_ms) {
            ins = ins->append_instance(action_t::STAY, 0, 0);
            if(timer.elapsed()!=time && timer.elapsed()%100==0) {
                time = timer.elapsed();
                cout << time << "	" << counter << endl;
            }
            ++counter;
        }
        delete ins;
    }

    // without assignment (with container)
    cout << endl << endl << "# without assignment (with container)" << endl;
    for(int create_counter=0 ; create_counter<items_to_create; ++create_counter) {
        timer.restart();
        Instance * ins = Instance::create(action_t::STAY, 0, 0);
        ins->set_container();
        int counter = 1;
        int time = timer.elapsed();
        cout << 0 << "	" << 0 << endl;
        while(timer.elapsed()<min_ms) {
            ins->append_instance(action_t::STAY, 0, 0);
            if(timer.elapsed()!=time && timer.elapsed()%100==0) {
                time = timer.elapsed();
                cout << time << "	" << counter << endl;
            }
            ++counter;
        }
        delete ins;
    }

    // with assignment (with container)
    cout << endl << endl << "# with assignment (with container)" << endl;
    for(int create_counter=0 ; create_counter<items_to_create; ++create_counter) {
        timer.restart();
        Instance * ins = Instance::create(action_t::STAY, 0, 0);
        ins->set_container();
        int counter = 1;
        int time = timer.elapsed();
        cout << 0 << "	" << 0 << endl;
        while(timer.elapsed()<min_ms) {
            ins = ins->append_instance(action_t::STAY, 0, 0);
            if(timer.elapsed()!=time && timer.elapsed()%100==0) {
                time = timer.elapsed();
                cout << time << "	" << counter << endl;
            }
            ++counter;
        }
        delete ins;
    }



    //=====================================================//
    //     Creating Container, Iterating, Random Access    //
    //=====================================================//

    int items_to_iterate = 1000000;
    int steps = 10000;
    int max_ms = 20000;

    cout << endl << endl << "# Creating container, iterating through it, and random accesing (sum of ten) item for at most " << items_to_iterate << " items" << endl;
    cout << "# item	time/ms (create from last)	time/ms (create from first)	time/ms (iterate with container)	time/ms (random with container)	time/ms (iterate without container)	time/ms (random without container)" << endl << endl;

    Instance * ins = Instance::create(action_t::STAY, 0, 0);
    int counter = 1;
    cout << 0 << "	" << 0 << "	" << 0 << "	" << 0 << "	" << 0 << "	" << 0 << "	" << 0 << endl;
    while(counter<items_to_iterate && timer.elapsed()<max_ms) {
        do {
            ins = ins->append_instance(action_t::STAY, 0, 0);
            ++counter;
        } while(counter%steps!=0);
        // set from last
        ins=ins->last();
        ins->unset_container();
        timer.restart();
        ins->set_container();
        cout << counter << "	" << timer.elapsed();
        // set from first
        ins=ins->first();
        ins->unset_container();
        timer.restart();
        ins->set_container();
        cout << "	" << timer.elapsed();
        // iterate with container
        timer.restart();
        for(InstanceIt insIt=ins->first(); insIt!=INVALID; ++insIt) {}
        cout << "	" << timer.elapsed();
        // random with container
        int random_time = 0;
        for(int rand_count=0; rand_count<10; ++rand_count) {
            timer.restart();
            ins->first()+rand()%(counter-1);
            random_time += timer.elapsed();
        }
        cout << "	" << random_time;
        // iterate without container
        ins->unset_container();
        timer.restart();
        for(InstanceIt insIt=ins->first(); insIt!=INVALID; ++insIt) {}
        cout << "	" << timer.elapsed();
        // random without container
        random_time = 0;
        for(int rand_count=0; rand_count<10; ++rand_count) {
            timer.restart();
            ins->first()+rand()%(counter-1);
            random_time += timer.elapsed();
        }
        cout << "	" << random_time << endl;
    }

    // cout << "# Iterating over " << items_to_iterate << " items" << endl;
    // cout << "# item	time/ms" << endl;

    // // without assignment (no container)
    // cout << endl << endl << "# without assignment (no container)" << endl;
    // for(int create_counter=0 ; create_counter<items_to_create; ++create_counter) {
    //     timer.restart();
    //     Instance * ins = Instance::create(action_t::STAY, 0, 0);
    //     int counter = 1;
    //     int time = timer.elapsed();
    //     while(timer.elapsed()<min_ms) {
    //         ins->append_instance(action_t::STAY, 0, 0);
    //         if(timer.elapsed()!=time && timer.elapsed()%100==0) {
    //             time = timer.elapsed();
    //             cout << time << "	" << counter << endl;
    //         }
    //         ++counter;
    //     }
    //     delete ins;
    // }

    return;

    // ins->unset_container();

    // int counter;

    // timer.restart();
    // counter = -1;
    // for(InstanceIt instance=ins->first(); instance!=util::INVALID; ++instance) {
    //     // DEBUG_OUT(0,"Instance: " << (*instance));
    //     ++counter;
    // }
    // DEBUG_OUT(0,"Loop of " << counter << " iterations (without container) took " << timer.elapsed() << " milliseconds.");

    // timer.restart();
    // ins->set_container();
    // DEBUG_OUT(0,"Took " << timer.elapsed() << " milliseconds to set up container.");

    // timer.restart();
    // counter = -1;
    // for(InstanceIt instance=ins->first(); instance!=util::INVALID; ++instance) {
    //     // DEBUG_OUT(0,"Instance: " << (*instance));
    //     ++counter;
    // }
    // DEBUG_OUT(0,"Loop of " << counter << " iterations (with container) took " << timer.elapsed() << " milliseconds.");

    // timer.restart();
    // ins->unset_container();
    // DEBUG_OUT(0,"Took " << timer.elapsed() << " milliseconds to unset container.");
}

Instance::Instance(const Instance& i):
    action(i.action),
    state(i.state),
    reward(i.reward),
    all(this),
    const_all(this),
    previous_instance(i.previous_instance),
    next_instance(i.next_instance),
    const_previous_instance(i.const_previous_instance),
    const_next_instance(i.const_next_instance),
    container(i.container),
    container_idx(i.container_idx)
{}

Instance::Instance(
    const action_t& a,
    const state_t& s,
    const reward_t& r
    ):
    action(a),
    state(s),
    reward(r),
    all(this),
    const_all(this),
    previous_instance(nullptr),
    next_instance(nullptr),
    const_previous_instance(nullptr),
    const_next_instance(nullptr),
    container(nullptr),
    container_idx(-1)
{}

const Instance * Instance::get_previous() const {
    return (const_previous_instance!=nullptr ? const_previous_instance : previous_instance);
}

const Instance * Instance::get_next() const {
    return (const_next_instance!=nullptr ? const_next_instance : next_instance);
}

Instance * Instance::get_non_const_previous() const {
    return  previous_instance;
}

Instance * Instance::get_non_const_next() const {
    return next_instance;
}

void Instance::unset_container_elements() {
    if(container==nullptr) {
        DEBUG_OUT(1,"Container not set");
    } else {
        for(idx_t idx=0; idx<(idx_t)container->size(); ++idx) {
            Instance * ins = (*container)[idx];
            if(ins->container!=container) {
                DEBUG_OUT(0,"Error: Container of instance does not point to this container");
            } else if(ins->container_idx!=idx) {
                DEBUG_OUT(0,"Error: Instance in container has unmatching index");
            } else if(ins!=this) {
                ins->container = nullptr;
                ins->container_idx = -1;
            }
        }
        this->container = nullptr;
        this->container_idx = -1;
    }
}

void Instance::fill_container(Instance * ins) {
    DEBUG_OUT(2,"Filling container..." );

    // get index and container
    idx_t idx = ins->container_idx;
    container_t * used_container = ins->container;

    // make sure container is large enough
    if((idx_t)used_container->size()<=idx) {
        used_container->resize(idx);
    }

    // fill container
    do {
        if(ins->container!=nullptr && ins->container!=used_container) {
            DEBUG_OUT(0,"Error: Instance already has a different container assigned");
        } else if(ins->container==nullptr && ins->container_idx!=-1) {
            DEBUG_OUT(0,"Error: Instance has no container assigned but container index other than -1");
        } else {
            DEBUG_OUT(2,"Setting " << (*ins) );
            ins->container = used_container;
            ins->container_idx = idx;
            if((idx_t)used_container->size()>idx) {
                (*(used_container))[idx] = ins;
            } else {
                used_container->push_back(ins);
            }
        }
        ++idx;
        ins = ins->next_instance;
    } while(ins!=nullptr);

    // shrink container if too large
    used_container->resize(idx);
}

#undef DEBUG_STRING
#define DEBUG_STRING "InstanceIt: "

InstanceIt::InstanceIt():
    util::InvalidAdapter<InstanceIt>(true),
    this_instance(nullptr)
{}

InstanceIt::InstanceIt(Instance * i):
    util::InvalidAdapter<InstanceIt>(false),
    this_instance(i)
{
    if(this_instance==nullptr) {
        this->invalidate();
    }
}

InstanceIt::operator Instance*() const {
    return this_instance;
}

InstanceIt::operator ConstInstanceIt() const {
    return ConstInstanceIt(this_instance);
}

Instance * InstanceIt::operator->() {
    return this_instance;
}

InstanceIt & InstanceIt::operator++() {
    if(this_instance==nullptr || this_instance->get_non_const_next()==nullptr) {
        this->invalidate();
    } else {
        this_instance = this_instance->get_non_const_next();
        DEBUG_OUT(2,"Go forward one step to " << *this_instance);
    }
    return *this;
}

InstanceIt & InstanceIt::operator--() {
    if(this_instance==nullptr || this_instance->get_non_const_previous()==nullptr) {
        this->invalidate();
    } else {
        this_instance = this_instance->get_non_const_previous();
        DEBUG_OUT(2,"Go backwards one step to " << *this_instance);
    }
    return *this;
}

InstanceIt & InstanceIt::operator+=(const int& c) {
    if(c==0 || *this==INVALID) {
        return (*this);
    } else if(c<0) {
        return (*this) -= -c;
    } else if(this_instance==nullptr) {
        this->invalidate();
        return (*this);
    } else {
        if(this_instance->container!=nullptr) { // use container (random access)
            if(this_instance->container_idx+c>=(idx_t)this_instance->container->size()) {
                this->invalidate();
            } else {
                (*this)=InstanceIt((*(this_instance->container))[this_instance->container_idx+c]);
                DEBUG_OUT(2,"Jump forward " << c << " steps to " << *this_instance);
            }
        } else { // use standard method (iterative pointer)
            ++(*this);
            if((*this)==INVALID) {
                return (*this);
            } else {
                return (*this)+=(c-1);
            }
        }
        return (*this);
    }
}

InstanceIt & InstanceIt::operator-=(const int& c) {
    if(c==0 || *this==INVALID) {
        return (*this);
    } else if(c<0) {
        return (*this) += -c;
    } else if(this_instance==nullptr) {
        this->invalidate();
        return (*this);
    } else {
        if(this_instance->container!=nullptr) { // use container (random access)
            if(this_instance->container_idx<c) {
                this->invalidate();
            } else {
                (*this)=InstanceIt((*(this_instance->container))[this_instance->container_idx-c]);
                DEBUG_OUT(2,"Jump backwards " << c << " steps to " << *this_instance);
            }
        } else { // use standard method (iterative pointer)
            --(*this);
            if((*this)==INVALID) {
                return (*this);
            } else {
                return (*this)-=(c-1);
            }
        }
        return (*this);
    }
}

InstanceIt InstanceIt::operator+(const int& c) const {
    return InstanceIt(*this)+=c;
}

InstanceIt InstanceIt::operator-(const int& c) const {
    return InstanceIt(*this)-=c;
}

int InstanceIt::length_to_first() const {
    if(this_instance!=nullptr && this_instance->container!=nullptr) {
        return this_instance->container_idx;
    } else {
        int counter = -1;
        for(InstanceIt ins=(*this); ins!=INVALID; --ins) {
            ++counter;
        }
        return counter;
    }
}

int InstanceIt::length_to_last() const {
    if(this_instance!=nullptr && this_instance->container!=nullptr) {
        return this_instance->container->size() - this_instance->container_idx - 1;
    } else {
        int counter = -1;
        for(InstanceIt ins=(*this); ins!=INVALID; ++ins) {
            ++counter;
        }
        return counter;
    }
}

#undef DEBUG_STRING
#define DEBUG_STRING "ConstInstanceIt: "

ConstInstanceIt::ConstInstanceIt():
    util::InvalidAdapter<ConstInstanceIt>(true),
    this_instance(nullptr)
{}

ConstInstanceIt::ConstInstanceIt(const Instance * i):
    util::InvalidAdapter<ConstInstanceIt>(false),
    this_instance(i)
{
    if(this_instance==nullptr) {
        this->invalidate();
    }
}

ConstInstanceIt::operator const Instance*() const {
    return this_instance;
}

const Instance * ConstInstanceIt::operator->() {
    return this_instance;
}

ConstInstanceIt & ConstInstanceIt::operator++() {
    if(this_instance==nullptr || this_instance->get_next()==nullptr) {
        this->invalidate();
    } else {
        this_instance = this_instance->get_next();
        DEBUG_OUT(2,"Go forward one step to " << *this_instance);
    }
    return *this;
}

ConstInstanceIt & ConstInstanceIt::operator--() {
    if(this_instance==nullptr || this_instance->get_previous()==nullptr) {
        this->invalidate();
    } else {
        this_instance = this_instance->get_previous();
        DEBUG_OUT(2,"Go backwards one step to " << *this_instance);
    }
    return *this;
}

ConstInstanceIt & ConstInstanceIt::operator+=(const int& c) {
    if(c==0 || *this==INVALID) {
        return (*this);
    } else if(c<0) {
        return (*this) -= -c;
    } else if(this_instance==nullptr) {
        this->invalidate();
        return (*this);
    } else {
        if(this_instance->container!=nullptr && this_instance->container_idx!=(idx_t)this_instance->container->size()-1) { // use container (random access)
            if(this_instance->container_idx+c>=(idx_t)this_instance->container->size()) {
                int rest = c - this_instance->container->size() + this_instance->container_idx + 1;
                (*this)=ConstInstanceIt(this_instance->container->back());
                DEBUG_OUT(2,"Jump forward " << rest << " steps to " << *this_instance);
                return (*this)+=rest;
            } else {
                (*this)=ConstInstanceIt((*(this_instance->container))[this_instance->container_idx+c]);
                return (*this);
            }
        } else { // use standard method (iterative pointer)
            ++(*this);
            if((*this)==INVALID) {
                return (*this);
            } else {
                return (*this)+=(c-1);
            }
        }
    }
}

ConstInstanceIt & ConstInstanceIt::operator-=(const int& c) {
    if(c==0 || *this==INVALID) {
        return (*this);
    } else if(c<0) {
        return (*this) += -c;
    } else if(this_instance==nullptr) {
        this->invalidate();
        return (*this);
    } else {
        if(this_instance->container!=nullptr && this_instance->container_idx!=0) { // use container (random access)
            DEBUG_OUT(2,"Using container to iterate");
            if(this_instance->container_idx - c < 0) {
                int rest = c - this_instance->container_idx;
                (*this)=ConstInstanceIt(this_instance->container->front());
                DEBUG_OUT(2,"Jump backwards " << rest << " steps to " << *this_instance);
                return (*this)-=rest;
            } else {
                (*this)=ConstInstanceIt((*(this_instance->container))[this_instance->container_idx-c]);
                return (*this);
            }
        } else { // use standard method (iterative pointer)
            --(*this);
            if((*this)==INVALID) {
                return (*this);
            } else {
                return (*this)-=(c-1);
            }
        }
    }
}

ConstInstanceIt ConstInstanceIt::operator+(const int& c) const {
    return ConstInstanceIt(*this)+=c;
}

ConstInstanceIt ConstInstanceIt::operator-(const int& c) const {
    return ConstInstanceIt(*this)-=c;
}

int ConstInstanceIt::length_to_first() const {
    int counter = -1;
    ConstInstanceIt tmpIt(this_instance); // invalid if this_instance==nullptr
    while(tmpIt!=INVALID) {
        if(tmpIt.this_instance->container!=nullptr) { // use container for jumps
            int steps = tmpIt.this_instance->container_idx+1;
            counter += steps;
            tmpIt -= steps;
        } else { // make single steps
            ++counter;
            --tmpIt;
        }
    }
    return counter;
}

int ConstInstanceIt::length_to_last() const {
    int counter = -1;
    ConstInstanceIt tmpIt(this_instance); // invalid if this_instance==nullptr
    while(tmpIt!=INVALID) {
        if(tmpIt.this_instance->container!=nullptr) { // use container for jumps
            int steps = tmpIt.this_instance->container->size() - tmpIt.this_instance->container_idx;
            counter += steps;
            tmpIt += steps;
        } else { // make single steps
            ++counter;
            ++tmpIt;
        }
    }
    return counter;
}

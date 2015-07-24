#include "GamblingHall.h"

#include <util/util.h>
#define DEBUG_LEVEL 0
#include <util/debug.h>

//#define VARIANT 0 // go to a machine or play current machine, infinitely often
#define VARIANT 1 // go to a machine within the first n steps, then play it once

#if VARIANT == 0

GamblingHall::GamblingHall(int machine_n, double tolerance):
    Environment({},{}), // set actions and states later!!
    machine_n(machine_n),
    tolerance(tolerance)
{
    if(tolerance<0) {
        DEBUG_ERROR("Tolerance should be in [0,1] but is " << tolerance);
        tolerance = 0;
    }
    if(tolerance>1) {
        DEBUG_ERROR("Tolerance should be in [0,1] but is " << tolerance);
        tolerance = 1;
    }
    // first state is: outside the gambling hall
    states.push_back(0);
    // first action is: playing the machine you're standing in front of
    actions.push_back(0);
    for(int machine : util::Range(1,machine_n)) {
        // state is: standing in front of that machine
        states.push_back(machine);
        // action is: going to that machine
        actions.push_back(machine);
    }
}

GamblingHall::state_reward_pair_t GamblingHall::sample(const state_t & s,
                                                       const action_t & a) const {
    reward_t reward;
    state_t state;
    if(s!=0 && a==0) {
        // you're inside the gambling hall and play a machine

        // p = (1/2 - tolerance/2) for s == 1
        // p = (1/2 + tolerance/2) for s == (number of machines)
        double p = (1. - tolerance)/2. + tolerance*(s-1)/(machine_n-1);
        reward = drand48()<p?1:0;
        state = s;
    } else {
        // you're outside the gambling hall and/or go to a machine
        reward = 0;
        state = a;
    }
    return state_reward_pair_t(state, reward);
}

bool GamblingHall::has_terminal_state() const {
    return false;
}

bool GamblingHall::is_terminal_state(state_t s) const {
    return false;
}

QString GamblingHall::state_name(const state_t & s) const {
    // just the default
    return Environment::state_name(s);
}

#elif VARIANT == 1

bool GamblingHall::GamblingHallAction::operator==(const Action & other) const {
    auto other_action = dynamic_cast<const GamblingHallAction*>(&other);
    return other_action!=nullptr && other_action->action==action;
}

size_t GamblingHall::GamblingHallAction::get_hash() const {
    return std::hash<int>()(action);
}

void GamblingHall::GamblingHallAction::write(std::ostream & out) const {
    out << action;
}

bool GamblingHall::GamblingHallObservation::operator==(const Observation & other) const {
    auto other_observation = dynamic_cast<const GamblingHallObservation*>(&other);
    return other_observation!=nullptr && other_observation->observation==observation;
}

size_t GamblingHall::GamblingHallObservation::get_hash() const {
    return std::hash<int>()(observation);
}

void GamblingHall::GamblingHallObservation::write(std::ostream & out) const {
    auto machine_and_time = util::convert_1D_to_ND_index(observation,{machine_n,time_n});
    out << "(m=" <<  machine_and_time[0] <<
        ", t=" << machine_and_time[1] << ")";
    // out << observation;
}

GamblingHall::GamblingHall(int machine_n, double tolerance):
//    Environment({-1,0,1},util::range_vector(machine_n*machine_n)),
    machine_n(machine_n),
    time_n(machine_n),
    tolerance(tolerance)
{
    if(tolerance<0) {
        DEBUG_ERROR("Tolerance should be in [0,1] but is " << tolerance);
        tolerance = 0;
    }
    if(tolerance>1) {
        DEBUG_ERROR("Tolerance should be in [0,1] but is " << tolerance);
        tolerance = 1;
    }
}

GamblingHall::observation_reward_pair_t GamblingHall::transition(const action_handle_t & action_handle) {
    auto gamblinghallaction = std::dynamic_pointer_cast<const GamblingHallAction>(action_handle);
    DEBUG_EXPECT(gamblinghallaction!=nullptr);
    int action = gamblinghallaction->action;
    reward_t reward;
    auto machine_and_time = util::convert_1D_to_ND_index(state,{machine_n,time_n});
    int machine = machine_and_time[0];
    const int time = machine_and_time[1];
    if(time<time_n-2) {
        // you move to another machine
        machine = util::clamp(0,machine_n-1,machine+action);
        reward = 0;
    } else {
        // you move to another machine
        machine = util::clamp(0,machine_n-1,machine+action);
        // and you play that machine
        // p = (1/2 - tolerance/2) for machine == 0
        // p = (1/2 + tolerance/2) for machine == (number of machines - 1)
        double p = (1. - tolerance)/2. + tolerance*machine/(machine_n-1);
        reward = drand48()<p?1:0;
    }
    state = util::convert_ND_to_1D_index({machine,time+1},{machine_n,time_n});
    observation_handle_t observation(new GamblingHallObservation(state,machine_n,time_n));
    DEBUG_OUT(1,"Transition: " << *action_handle << " --> " << *observation << " (" << reward << ")");
    return observation_reward_pair_t(observation, reward);
}

GamblingHall::action_container_t GamblingHall::get_actions() {
    return action_container_t({action_handle_t(new GamblingHallAction(-1)),
                action_handle_t(new GamblingHallAction(0)),
                action_handle_t(new GamblingHallAction(1))});
}

void GamblingHall::make_current_state_default() {
    default_state = state;
}

void GamblingHall::reset_state() {
    state = default_state;
}

bool GamblingHall::has_terminal_state() const {
    return true;
}

bool GamblingHall::is_terminal_state() const {
    return util::convert_1D_to_ND_index(state,{machine_n,time_n})[1]==time_n-1;
}

#endif

void GamblingHall::write(std::ostream & out) const {
    out << "GamblingHall(machine_n=" << machine_n
        << ";time_n=" << time_n
        << ";tolerance=" << tolerance << ")";
}

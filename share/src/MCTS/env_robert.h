/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#pragma once

#include "environment.h"
#include "../../include/MCTS_Environment/AbstractEnvironment.h"


class InterfaceMarc: public AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct InterfaceMarcAction: public Action {
        InterfaceMarcAction(MCTS_Environment::Handle action): action(action) {}
        virtual bool operator==(const Action & other) const {
            auto interface_action = dynamic_cast<const InterfaceMarcAction *>(&other);
            return interface_action!=nullptr && *(interface_action->action)==*action;
        }
        virtual size_t get_hash() const {
            return action->get_hash();
        }
        virtual void write(std::ostream & out) const {
            action->write(out);
        }
        MCTS_Environment::Handle action;
    };
    struct InterfaceMarcObservation: public Observation {
        InterfaceMarcObservation(MCTS_Environment::Handle observation): observation(observation) {}
        virtual bool operator==(const Observation & other) const {
            auto interface_observation = dynamic_cast<const InterfaceMarcObservation *>(&other);
            return interface_observation!=nullptr && *(interface_observation->observation)==*(observation);
        }
        virtual size_t get_hash() const {
            return observation->get_hash();
        }
        virtual void write(std::ostream & out) const {
            observation->write(out);
        }
        MCTS_Environment::Handle observation;
    };

    //----members----//
public:
    std::shared_ptr<MCTS_Environment> env_marc;

    //----methods----//
public:
    InterfaceMarc(std::shared_ptr<MCTS_Environment> env_marc): env_marc(env_marc) {}
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) {
        auto interface_action = std::dynamic_pointer_cast<const InterfaceMarcAction>(action_handle);
        assert(interface_action!=nullptr);
        auto return_value = env_marc->transition(interface_action->action);
        return observation_reward_pair_t(observation_handle_t(new InterfaceMarcObservation(return_value.first)),return_value.second);
    }
    template<class C>
        static std::shared_ptr<AbstractEnvironment> makeAbstractEnvironment(C * env) {
        auto mcts = dynamic_cast<MCTS_Environment*>(env);
        assert(mcts!=nullptr);
        return std::shared_ptr<AbstractEnvironment>(
            std::make_shared<InterfaceMarc>(
                std::shared_ptr<MCTS_Environment>(mcts)));
    }
    virtual action_container_t get_actions() {
        action_container_t action_container;
        for(auto action : env_marc->get_actions()) {
            action_container.push_back(action_handle_t(new InterfaceMarcAction(action)));
        }
        return action_container;
    }
    virtual void make_current_state_new_start() {
        env_marc->make_current_state_new_start();
    }
    virtual void reset_state() {
        env_marc->reset_state();
    }




    virtual bool has_terminal_state() const {
        return env_marc->get_info(MCTS_Environment::InfoTag::hasTerminal);
    }
    virtual bool is_terminal_state() const {
        return env_marc->is_terminal_state();
    }
    virtual bool is_deterministic() const {
        return env_marc->get_info(MCTS_Environment::InfoTag::isDeterministic);
    }
    virtual bool has_max_reward() const {
        return env_marc->get_info(MCTS_Environment::InfoTag::hasMaxReward);
    }
    virtual reward_t max_reward() const {
        return (reward_t)env_marc->get_info_value(MCTS_Environment::InfoTag::getMaxReward);
    }
    virtual bool has_min_reward() const {
        return env_marc->get_info(MCTS_Environment::InfoTag::hasMinReward);
    }
    virtual reward_t min_reward() const {
        return (reward_t)env_marc->get_info_value(MCTS_Environment::InfoTag::getMinReward);
    }
    virtual bool is_markov() const {
        return env_marc->get_info(MCTS_Environment::InfoTag::isMarkov);
    }
};


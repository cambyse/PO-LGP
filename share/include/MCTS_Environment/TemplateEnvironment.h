#ifndef TEMPLATEENVIRONMENT_H_
#define TEMPLATEENVIRONMENT_H_

#include <MCTS_Environment/AbstractEnvironment.h>

class TemplateEnvironment: public AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct TemplateAction: public Action {
        virtual bool operator==(const Action & other) const override;
        virtual size_t get_hash() const override;
        virtual void write(std::ostream &) const override;
    };
    struct TemplateObservation: public Observation {
        virtual bool operator==(const Observation & other) const override;
        virtual size_t get_hash() const override;
        virtual void write(std::ostream &) const override;
    };
    struct TemplateState: public State {};

    //----members----//
    //      ...      //

    //----methods----//
public:
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override;
    virtual action_container_t get_actions() override;
    virtual state_handle_t get_state_handle() override;
    virtual void set_state(const state_handle_t & state_handle) override;
    virtual bool has_terminal_state() const override;
    virtual bool is_terminal_state() const override;
    virtual bool is_deterministic() const override;
    virtual bool has_max_reward() const override;
    virtual reward_t max_reward() const override;
    virtual bool has_min_reward() const override;
    virtual reward_t min_reward() const override;
    virtual bool is_markov() const override;
};

#endif /* TEMPLATEENVIRONMENT_H_ */

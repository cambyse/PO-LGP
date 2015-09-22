#ifndef GAMBLINGHALL_H_
#define GAMBLINGHALL_H_

#include <MCTS_Environment/AbstractEnvironment.h>

class GamblingHall: public AbstractEnvironment {
    //----typedefs/classes----//
    public:
    struct GamblingHallAction: public Action {
        GamblingHallAction(int action): action(action) {}
        virtual bool operator==(const Action & other) const override;
        virtual size_t get_hash() const override;
        virtual void write(std::ostream &) const override;
        int action;
    };
    struct GamblingHallObservation: public Observation {
        GamblingHallObservation(int observation, int machine_n, int time_n):
            observation(observation), machine_n(machine_n), time_n(time_n) {}
        virtual bool operator==(const Observation & other) const override;
        virtual size_t get_hash() const override;
        virtual void write(std::ostream &) const override;
        int observation;
        int machine_n, time_n;
    };

    //----members----//
private:
    int machine_n, time_n;
    double tolerance;
    int default_state = 0;
    int state = 0;

    //----methods----//
public:
    GamblingHall(int machine_n, double tolerance = 0.1);
    virtual ~GamblingHall() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override;
    virtual action_container_t get_actions() override;
    virtual void make_current_state_default() override;
    virtual void reset_state() override;
    virtual bool has_terminal_state() const override;
    virtual bool is_terminal_state() const override;
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual bool is_markov() const override {return true;}
    virtual void write(std::ostream &) const override;
};

#endif /* GAMBLINGHALL_H_ */

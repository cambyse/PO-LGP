#ifndef RANDOMSEARCH_H_
#define RANDOMSEARCH_H_

#include "AbstractSearchTree.h"

#include <deque>

class RandomSearch: public AbstractSearchTree {
    //----typedefs/classes----//
    typedef std::tuple<action_handle_t, observation_handle_t, reward_t> transition_t;
    typedef std::deque<transition_t> rollout_t;

    //----members----//
    action_handle_t best_action;
    reward_t best_return;
    rollout_t best_rollout;

    //----methods----//
public:
    RandomSearch(std::shared_ptr<AbstractEnvironment> environment,
                 double discount);
    virtual ~RandomSearch() = default;
    virtual void init() override;
    virtual void next() override;
    virtual action_handle_t recommend_action() const override;
    virtual void update(const action_handle_t &,
                       const observation_handle_t &) override;
    virtual void plot_graph(const char* file_name,
                            const char* command = "dot",
                            const char* parameters = "-Tpdf",
                            bool delete_dot_file = true) const override;
    virtual void write(std::ostream &) const override;
};

#endif /* RANDOMSEARCH_H_ */

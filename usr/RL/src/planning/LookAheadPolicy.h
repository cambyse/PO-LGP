#ifndef LOOKAHEADPOLICY_H_
#define LOOKAHEADPOLICY_H_

#include "Policy.h"
#include "LookAheadSearch.h"
#include <environment/Predictor.h>
#include <memory> // for shared_ptr

class LookAheadPolicy: public Policy {
public:
    LookAheadPolicy(const double& discount,
                    std::shared_ptr<const Predictor> predictor,
                    const bool& prune = false,
                    const int& max_tree_size = 10000
        );
    virtual ~LookAheadPolicy() = default;
    virtual action_ptr_t get_action(const_instance_ptr_t) override;
    virtual void set_pruning(bool prune) { prune_tree = prune; }
    virtual void set_max_tree_size(int max) { max_tree_size = max; }
    virtual void invalidate_search_tree() { valid_tree_exists = false; }
    virtual void set_discount(double d) { look_ahead_search.set_discount(d); }
    virtual const LookAheadSearch & get_look_ahead_search() const { return look_ahead_search; }
private:
    LookAheadSearch look_ahead_search;
    std::shared_ptr<const Predictor> predictor;
    bool prune_tree;
    bool valid_tree_exists;
    action_ptr_t last_action;
    int max_tree_size;
};

#endif /* LOOKAHEADPOLICY_H_ */

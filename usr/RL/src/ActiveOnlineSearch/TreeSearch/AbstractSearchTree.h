#ifndef ABSTRACTSEARCHTREE_H_
#define ABSTRACTSEARCHTREE_H_

#include <memory> // for shared_ptr
#include <ostream> // for ostream
#include <MCTS_Environment/AbstractEnvironment.h>

class AbstractSearchTree {
    //----typedefs/classes----//
public:
    typedef AbstractEnvironment::action_handle_t      action_handle_t;
    typedef AbstractEnvironment::observation_handle_t observation_handle_t;
    typedef AbstractEnvironment::reward_t             reward_t;

    //----members----//
protected:
    /**
     * Pointer to the environment. */
    std::shared_ptr<AbstractEnvironment> environment;
    /**
     * Discount factor for computing the value. */
    double discount;

    //----methods----//
public:
    AbstractSearchTree(std::shared_ptr<AbstractEnvironment> environment,
                       const double discount):
        environment(environment),
        discount(discount) {}
    virtual ~AbstractSearchTree() = default;
    /**
     * Initializes an empty search tree. This function may be used to reset
     * everything and, depending on the implementation, may need to be called
     * before using the AbstractSearchTree. */
    virtual void init() = 0;
    /**
     * Proceed with planning. Performs the next iteration in the planning
     * process, such as a new rollout in Monte-Carlo Tree Search methods. */
    virtual void next() = 0;
    /**
     * Returns a recommendation for an action for the root node. */
    virtual action_handle_t recommend_action() const = 0;
    /**
     * Prunes the tree according to the given action and observation. This
     * function may be called after an action was actually performed in the
     * environment to reuse the relevant rest of the tree instead of resetting
     * it with init().*/
    virtual void update(const action_handle_t &,
                        const observation_handle_t &) = 0;
    /**
     * Prints the graph to a PDF file with given name. */
    virtual void plot_graph(const char* file_name,
                            const char* command = "dot",
                            const char* parameters = "-Tpdf",
                            bool delete_dot_file = true) const = 0;
    friend std::ostream& operator<<(std::ostream & out, const AbstractSearchTree & search) {
        search.write(out);
        return out;
    }
    /**
     * Print describtion. Ideally this should contain all relevant information
     * (like class name and parameters) that are needed to reproduce an
     * experiment. */
    virtual void write(std::ostream &) const = 0;
};

#endif /* ABSTRACTSEARCHTREE_H_ */

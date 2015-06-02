#ifndef ACTIVETREESEARCH_H_
#define ACTIVETREESEARCH_H_

#include "SearchTree.h"

#include <vector>
#include <map>

class ActiveTreeSearch: public SearchTree {
    //----typedefs/classes----//
    typedef std::map<node_t,node_t> node_array_t;
    typedef std::map<node_t,node_array_t> node_matrix_t;
    struct VariableInfo {
        node_t pi, mean_Q, var_Q, mean_r, var_r, A, B, C;
        node_array_t mean_p, alpha, beta;
        node_matrix_t var_p, gamma;
    };

    //----members----//
protected:
    graph_t c_graph;
    graph_t::NodeMap<VariableInfo> variable_info_map;
    graph_t::NodeMap<node_t> base_node;
    graph_t::NodeMap<QString> c_node_name;

    //----methods----//
public:
    ActiveTreeSearch(std::shared_ptr<AbstractEnvironment> environment,
                     double discount,
                     std::shared_ptr<NodeFinder> node_finder);
    virtual ~ActiveTreeSearch() = default;
    virtual void next_do() override;
    virtual action_handle_t recommend_action() const override;
    virtual void init(const state_handle_t & root_state) override;
    virtual void toPdf(const char* file_name) const override;
    virtual arc_node_t add_observation_node(observation_handle_t observation, node_t action_node) override;
    virtual arc_node_t add_action_node(action_handle_t action, node_t observation_node) override;
    virtual void erase_node(node_t) override;
};

#endif /* ACTIVETREESEARCH_H_ */

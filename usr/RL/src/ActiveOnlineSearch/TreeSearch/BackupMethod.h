#ifndef BACKUPMETHOD_H_
#define BACKUPMETHOD_H_

#include <memory>

#include "AbstractMonteCarloTreeSearch.h"

#include "TreePolicy.h"

class Environment;

namespace backup_method {

    typedef AbstractMonteCarloTreeSearch::graph_t              graph_t;
    typedef AbstractMonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::mcts_arc_info_map_t  mcts_arc_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_info_map_t      node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_t               node_t;
    typedef AbstractMonteCarloTreeSearch::arc_t                arc_t;
    typedef AbstractMonteCarloTreeSearch::out_arc_it_t         out_arc_it_t;
    typedef AbstractMonteCarloTreeSearch::in_arc_it_t          in_arc_it_t;
    typedef AbstractEnvironment::action_handle_t               action_handle_t;
    typedef AbstractEnvironment::observation_handle_t          observation_handle_t;
    typedef AbstractEnvironment::state_handle_t                state_handle_t;
    typedef AbstractEnvironment::reward_t                      reward_t;

    /**
     * Abstract basis class for backup methods. For each internal node that was
     * visited a backup will be performed. This usually are either Monte-Carlo
     * backups using rollouts from that node or dynamic programming backups. */
    class BackupMethod {
    public:
        virtual void operator()(const node_t & state_node,
                                const node_t & action_node,
                                double discount,
                                std::shared_ptr<Environment> environment,
                                const graph_t & graph,
                                const node_info_map_t & node_info_map,
                                mcts_node_info_map_t & mcts_node_info_map,
                                const mcts_arc_info_map_t & mcts_arc_info_map) const = 0;
    };

    /**
     * Performs Bellman backups. Currently rewards are assumed to depend only on
     * the source state and the action, not on the tartet state! */
    class Bellman: public BackupMethod {
    public:
        Bellman(std::shared_ptr<const tree_policy::TreePolicy> tree_policy = nullptr);
        virtual void operator()(const node_t & state_node,
                                const node_t & action_node,
                                double discount,
                                std::shared_ptr<Environment> environment,
                                const graph_t & graph,
                                const node_info_map_t & node_info_map,
                                mcts_node_info_map_t & mcts_node_info_map,
                                const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    protected:
        std::shared_ptr<const tree_policy::TreePolicy> tree_policy;
    };

    /**
     * Performs Monte-Carlo backups. */
    class MonteCarlo: public BackupMethod {
    public:
        virtual void operator()(const node_t & state_node,
                                const node_t & action_node,
                                double discount,
                                std::shared_ptr<Environment> environment,
                                const graph_t & graph,
                                const node_info_map_t & node_info_map,
                                mcts_node_info_map_t & mcts_node_info_map,
                                const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    };

} // namespace backup_method

#endif /* BACKUPMETHOD_H_ */

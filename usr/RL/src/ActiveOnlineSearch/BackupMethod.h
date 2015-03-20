#ifndef BACKUPMETHOD_H_
#define BACKUPMETHOD_H_

#include "AbstractMonteCarloTreeSearch.h"

class Environment;

namespace backup_method {

    typedef AbstractMonteCarloTreeSearch::graph_t              graph_t;
    typedef AbstractMonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::mcts_arc_info_map_t  mcts_arc_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_info_map_t      node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_t               node_t;
    typedef AbstractMonteCarloTreeSearch::arc_t                arc_t;
    typedef AbstractMonteCarloTreeSearch::out_arc_it_t         out_arc_it_t;
    typedef AbstractMonteCarloTreeSearch::action_t             action_t;
    typedef AbstractMonteCarloTreeSearch::reward_t             reward_t;

    /**
     * Abstract basis class for backup methods. For each internal node that was
     * visited a backup will be performed. This usually are either Monte-Carlo
     * backups using rollouts from that node or dynamic programming backups. */
    class BackupMethod {
    public:
        virtual void operator()(const node_t & state_node,
                                const node_t & action_node,
                                double discount,
                                const Environment & environment,
                                const graph_t & graph,
                                mcts_node_info_map_t & mcts_node_info_map,
                                const mcts_arc_info_map_t & mcts_arc_info_map) const = 0;
    };

    /**
     * Performs Bellman backups. */
    class Bellman: public BackupMethod {
    public:
        virtual void operator()(const node_t & state_node,
                                const node_t & action_node,
                                double discount,
                                const Environment & environment,
                                const graph_t & graph,
                                mcts_node_info_map_t & mcts_node_info_map,
                                const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    };

} // namespace backup_method

#endif /* BACKUPMETHOD_H_ */

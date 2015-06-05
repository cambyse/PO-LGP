#ifndef BACKUPMETHOD_H_
#define BACKUPMETHOD_H_

#include <memory>

#include "MonteCarloTreeSearch.h"

#include "TreePolicy.h"

namespace backup_method {

    typedef MonteCarloTreeSearch::graph_t              graph_t;
    typedef MonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef MonteCarloTreeSearch::mcts_arc_info_map_t  mcts_arc_info_map_t;
    typedef MonteCarloTreeSearch::node_info_map_t      node_info_map_t;
    typedef MonteCarloTreeSearch::node_t               node_t;
    typedef MonteCarloTreeSearch::arc_t                arc_t;
    typedef MonteCarloTreeSearch::out_arc_it_t         out_arc_it_t;
    typedef MonteCarloTreeSearch::in_arc_it_t          in_arc_it_t;
    typedef AbstractEnvironment::action_handle_t       action_handle_t;
    typedef AbstractEnvironment::observation_handle_t  observation_handle_t;
    typedef AbstractEnvironment::state_handle_t        state_handle_t;
    typedef AbstractEnvironment::reward_t              reward_t;

    /**
     * Abstract basis class for backup methods. For each internal node that was
     * visited a backup will be performed. This usually are either Monte-Carlo
     * backups using rollouts from that node or dynamic programming backups. */
    class BackupMethod {
    public:
        //----members----//
        double discount = 0;
        std::shared_ptr<AbstractEnvironment> environment;
        const graph_t * graph = nullptr;
        const node_info_map_t * node_info_map = nullptr;
        const mcts_arc_info_map_t * mcts_arc_info_map = nullptr;
    public:
        //----methods----//
        virtual void init(double discount,
                          std::shared_ptr<AbstractEnvironment> environment,
                          const graph_t & graph,
                          const node_info_map_t & node_info_map,
                          const mcts_arc_info_map_t & mcts_arc_info_map);
        virtual void backup(const node_t & observation_node,
                            const node_t & action_node,
                            mcts_node_info_map_t & mcts_node_info_map) const = 0;
        virtual void backup_root(const node_t & observation_node,
                                 mcts_node_info_map_t & mcts_node_info_map) const {};
    };

    /**
     * Performs Bellman backups. Currently rewards are assumed to depend only on
     * the source state and the action, not on the tartet state! */
    class Bellman: public BackupMethod {
    public:
        Bellman(std::shared_ptr<const tree_policy::TreePolicy> tree_policy = nullptr);
        virtual void backup(const node_t & observation_node,
                            const node_t & action_node,
                            mcts_node_info_map_t & mcts_node_info_map) const override;
    protected:
        std::shared_ptr<const tree_policy::TreePolicy> tree_policy;
    };

    /**
     * Performs Monte-Carlo backups. */
    class MonteCarlo: public BackupMethod {
    public:
        virtual void backup(const node_t & observation_node,
                            const node_t & action_node,
                            mcts_node_info_map_t & mcts_node_info_map) const override;
    };

} // namespace backup_method

#endif /* BACKUPMETHOD_H_ */

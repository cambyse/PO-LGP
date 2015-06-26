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
    typedef AbstractEnvironment::reward_t              reward_t;
    typedef AbstractEnvironment::action_container_t    action_container_t;

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
        mcts_node_info_map_t * mcts_node_info_map = nullptr;
        const mcts_arc_info_map_t * mcts_arc_info_map = nullptr;
    public:
        //----methods----//
        virtual ~BackupMethod() = default;
        virtual void init(double discount,
                          std::shared_ptr<AbstractEnvironment> environment,
                          const graph_t & graph,
                          const node_info_map_t & node_info_map,
                          mcts_node_info_map_t & mcts_node_info_map,
                          const mcts_arc_info_map_t & mcts_arc_info_map);
        virtual void backup_action_node(const node_t & action_node) const = 0;
        virtual void backup_observation_node(const node_t & observation_node) const = 0;
        virtual void backup_root(const node_t & observation_node) const {};
    };

    /**
     * Performs Bellman backups. Currently rewards are assumed to depend only on
     * the source state and the action, not on the tartet state! */
    class Bellman: public BackupMethod {
    public:
        Bellman(std::shared_ptr<tree_policy::TreePolicy> tree_policy = nullptr,
                double prior_counts = -1);
        virtual ~Bellman() = default;
        virtual void init(double discount,
                          std::shared_ptr<AbstractEnvironment> environment,
                          const graph_t & graph,
                          const node_info_map_t & node_info_map,
                          mcts_node_info_map_t & mcts_node_info_map,
                          const mcts_arc_info_map_t & mcts_arc_info_map) override;
        virtual void backup_action_node(const node_t & action_node) const override;
        virtual void backup_observation_node(const node_t & observation_node) const override;
    protected:
        std::shared_ptr<tree_policy::TreePolicy> tree_policy;
        double prior_counts;
    };

    /**
     * Performs Monte-Carlo backups. */
    class MonteCarlo: public BackupMethod {
    public:
        MonteCarlo(double prior_counts = -1);
        virtual ~MonteCarlo() = default;
        virtual void init(double discount,
                          std::shared_ptr<AbstractEnvironment> environment,
                          const graph_t & graph,
                          const node_info_map_t & node_info_map,
                          mcts_node_info_map_t & mcts_node_info_map,
                          const mcts_arc_info_map_t & mcts_arc_info_map) override;
        virtual void backup_action_node(const node_t & action_node) const override;
        virtual void backup_observation_node(const node_t & observation_node) const override;
    protected:
        virtual void backup_node(const node_t & node) const;
        double prior_counts;
    };

    /**
     * Performs hybrid Dynamic Programming (Bellman) and Monte-Carlo backups. MC
     * backups are weighted with \p mc_weight (given in constructor). That is
     * for \p mc_weight = 1 this corresponds to MonteCarlo backups and for
     * mc_weight = 0 this corresponds to Bellman backups. */
    class HybridMCDP: public BackupMethod {
    public:
        HybridMCDP(double mc_weight = 0.5,
                   double reward_prior_counts = -1,
                   double return_prior_counts = -1);
        virtual ~HybridMCDP() = default;
        virtual void init(double discount,
                          std::shared_ptr<AbstractEnvironment> environment,
                          const graph_t & graph,
                          const node_info_map_t & node_info_map,
                          mcts_node_info_map_t & mcts_node_info_map,
                          const mcts_arc_info_map_t & mcts_arc_info_map) override;
        virtual void backup_action_node(const node_t & action_node) const override;
        virtual void backup_observation_node(const node_t & observation_node) const override;
    protected:
        double mc_weight;
        MonteCarlo monte_carlo;
        Bellman bellman;
    };

} // namespace backup_method

#endif /* BACKUPMETHOD_H_ */

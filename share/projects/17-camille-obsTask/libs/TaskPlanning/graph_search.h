#pragma once

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <policy.h>
#include <task_planner.h>

#include <po_graph_node.h>
#include <node_visitors.h>

namespace tp
{

class GraphSearchPlanner : public TaskPlanner
{
public:

  // modifiers
  void setFol( const std::string & folDescription ) override;
  void solve() override;
  void integrate( const Policy::ptr & policy ) override;

  // getters
  Policy::ptr getPolicy() const override;
  MotionPlanningOrder getPlanningOrder() const override;
  bool      terminated () const override { return solved(); }

private:
  bool solved() const { return root_->isSolved(); }

private:
  void buildGraph();
  void dijkstra();
  void extractSolutions();

  uint n_exp_ = 0;

private:
  void checkIntegrity();

private:
  // state
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  arr bs_;

  POGraphNode::ptr root_;

  // graph building
  std::set< POGraphNode::ptr > checked_;
  std::queue< POGraphNode::ptr > queue_;
  std::list< POGraphNode::ptr > terminals_;

  // dijkstra
  std::vector< double > expectedReward_;

  // constants
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
};

}

#pragma once

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <policy.h>
#include <task_planner.h>

#include <po_graph.h>
#include <node_visitors.h>
#include <po_djikstra.h>

namespace tp
{

class GraphSearchPlanner : public TaskPlanner
{
public:

  // modifiers
  void setFol( const std::string & folDescription ) override;
  void buildGraph();
  void solve() override;
  void integrate( const Policy::ptr & policy ) override;

  // getters
  POGraph::ptr getGraph() const { return graph_; }
  mlr::Array< std::shared_ptr<FOL_World> > getFolEngines() const { return folEngines_; }
  Policy::ptr getPolicy() const override;
  MotionPlanningOrder getPlanningOrder() const override;
  bool      terminated () const override { return policy_ != nullptr; }

  // utility
  void saveGraphToFile( const std::string & filename );

private:
  void buildGraphImpl();
  void yen( uint k );   // generates a set of policies

  uint n_exp_ = 0;

private:
  void checkIntegrity();

private:
  // state
  mlr::Array< std::shared_ptr<FOL_World> > folEngines_;
  arr bs_;
  POGraph::ptr graph_;

  // dijkstra
  std::vector< double > expectedReward_;
  // policy reconstruction
  std::vector< int >   bestFamily_;     // action to take in this bs and i
  std::vector< POGraphNode::ptr > parents_;
  Policy::ptr policy_;
  std::map< POGraphNode::ptr, PolicyNode::ptr > PO2Policy_;

  // constants
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
};

}

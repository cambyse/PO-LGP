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

  // utility
  void saveGraphToFile( const std::string & filename );
private:
  bool solved() const { return root_->isSolved(); }

private:
  void buildGraph();
  void yen( uint k );   // generates a set of policies

  uint n_exp_ = 0;

private:
  void checkIntegrity();

private:
  POGraphNode::ptr root_;

  // state
  mlr::Array< std::shared_ptr<FOL_World> > folEngines_;
  arr bs_;

  // graph building
  //std::set  < POGraphNode::ptr > checked_;
  std::queue< POGraphNode::ptr > queue_;
  std::list < POGraphNode::ptr > terminals_;

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

class Yens
{
public:
  Yens( const POGraphNode::ptr & root, const mlr::Array< std::shared_ptr<FOL_World> > & folEngines );

  std::list< Policy::ptr > solve( const std::list < POGraphNode::ptr > & terminals, uint k );
private:
};

class Dijkstra
{
public:
  Dijkstra( const POGraphNode::ptr & root, const mlr::Array< std::shared_ptr<FOL_World> > & folEngines );

  Policy::ptr solve( const std::list < POGraphNode::ptr > & terminals );

private:
  void dijkstra( const std::list < POGraphNode::ptr > & terminals );
  void extractSolutions();
  void extractSolutionFrom( const POGraphNode::ptr & );
  void buildPolicy();
  void buildPolicyFrom( const POGraphNode::ptr & );

private:
  const POGraphNode::ptr & root_;

  mlr::Array< std::shared_ptr<FOL_World> > folEngines_;
  std::vector< double > expectedReward_;
  // policy reconstruction
  std::vector< int >   bestFamily_;     // action to take in this bs and i
  std::vector< POGraphNode::ptr > parents_;
  Policy::ptr policy_;
  std::map< POGraphNode::ptr, PolicyNode::ptr > PO2Policy_;
};

}

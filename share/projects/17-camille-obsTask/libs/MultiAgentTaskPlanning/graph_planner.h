#pragma once

#include <stdexcept>

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <new_policy.h>
#include <task_planner.h>

#include <logic_parser.h>
#include <decision_graph.h>

namespace matp
{

std::vector< std::string > decisionArtifactToKomoArgs( const std::string & artifact );

class GraphPlanner : public TaskPlanner
{
public:
  // modifiers
  virtual void setFol( const std::string & descrition ) override;
  virtual void solve() override;
  virtual void integrate( const Policy::ptr & policy ) override;

  // getters
  virtual bool terminated() const override;
  virtual Policy::ptr getPolicy() const override;
  NewPolicy getNewPolicy() const;

  virtual MotionPlanningOrder getPlanningOrder() const override;

  // other modifiers
  void setR0( double r0 ) { r0_ = r0; }
  void buildGraph( int maxSteps = -1 );
  void saveGraphToFile( const std::string & filename ) const { graph_.saveGraphToFile( filename ); }

  // other getters
  DecisionGraph decisionGraph() const { return graph_; }
  uint agentNumber() const { return parser_.agentNumber(); }

  // stand-alone
  NewPolicyNodeData decisionGraphtoPolicyData( const NodeData & n ) const;

private:
  void valueIteration();
  void decideOnDecisionGraphCopy();
  void buildPolicy();

private:
  LogicParser parser_;
  DecisionGraph graph_;
  NewPolicy policy_;

  // value iteration
  double r0_ = -1;
  std::vector< double > values_;
  DecisionGraph decidedGraph_;

};

} // namespace matp

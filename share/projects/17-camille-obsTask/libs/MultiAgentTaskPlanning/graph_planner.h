#pragma once

#include <stdexcept>

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <policy.h>
#include <task_planner.h>

#include <logic_parser.h>
#include <decision_graph.h>

namespace matp
{

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
  virtual MotionPlanningOrder getPlanningOrder() const override;

  // other modifiers
  void buildGraph( int maxSteps = -1 );
  DecisionGraph decisionGraph() const { return graph_; }

  // other getters
  uint agentNumber() const { return parser_.agentNumber(); }

private:
  LogicParser parser_;
  DecisionGraph graph_;
};

} // namespace matp

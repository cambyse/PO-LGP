#pragma once

#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <policy.h>
#include <task_planner.hpp>

#include <po_node.h>
#include <node_visitors.h>

namespace tp
{

class MCTSPlanner : public TaskPlanner
{
public:
  //virtual ~TaskPlanner();

  // modifiers
  void setFol( const std::string & folDescription ) override;
  void solve() override;
  void integrate( const Policy::ptr & policy ) override;

  // getters
  Policy::ptr getPolicy() const override;
  bool solved() const { return root_->isSolved(); }

private:
  PONode::L getNodesToExpand() const;   // go along the best solution so far and accumulates the nodes that haven't been expanded, it goes up to the "deepest nodes" of the temporary path
  PONode::L getNodesToExpand( const PONode::ptr & ) const;

private:
  // state
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  arr bs_;

  PONode::ptr root_; // root and "current" node

  // params
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
};

}

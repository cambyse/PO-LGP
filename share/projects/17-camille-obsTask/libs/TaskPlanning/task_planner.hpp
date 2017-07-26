#pragma once

#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <policy.h>

#include <po_node.h>
#include <node_visitors.h>

namespace tp
{

class TaskPlanner
{
public:
  //virtual ~TaskPlanner();

  // modifiers
  void setFol( const std::string & folDescription );
  void solve();

  // getters
  bool solved() const { return root_->isSolved(); }
  Policy::ptr getPolicy() const;

private:
  PONode::L getNodesToExpand() const;   // go along the best solution so far and accumulates the nodes that haven't been expanded, it goes up to the "deepest nodes" of the temporary path
  PONode::L getNodesToExpand( PONode::ptr ) const;

private:
  // state
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  arr bs_;

  PONode::ptr root_; // root and "current" node

  // params
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";

};

}

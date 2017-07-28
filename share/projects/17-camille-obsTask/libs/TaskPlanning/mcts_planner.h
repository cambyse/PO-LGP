#pragma once

#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <policy.h>
#include <task_planner.h>

#include <po_node.h>
#include <node_visitors.h>

namespace tp
{

class MCTSPlanner : public TaskPlanner
{
public:
  MCTSPlanner();
  //virtual ~TaskPlanner();

  // modifiers
  void setFol( const std::string & folDescription ) override;
  void solve() override;
  void integrate( const Policy::ptr & policy ) override;

  // getters
  Policy::ptr getPolicy() const override;
  bool terminated () const override { return false; } //

private:
  bool solved() const { return root_->isSolved(); }
  void integrateFromNode( const PONode::ptr &, const PolicyNode::ptr & );

private:
  void solveFirstTime();
  void generateAlternative();

  PONode::L getNodesToExpand() const;   // go along the best solution so far and accumulates the nodes that haven't been expanded, it goes up to the "deepest nodes" of the temporary path
  PONode::L getNodesToExpand( const PONode::ptr & ) const;

private:
  // state
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  arr bs_;

  PONode::ptr root_; // root and "current" node

  std::list< Policy::ptr > solutions_;

  // alternative generation
  bool currentPolicyFringeInitialized_;
  std::set< PONode::L > currentPolicyFringe_;
  PONode::ptr alternativeStartNode_;
  PONode::L nextFamilyBackup_;
  std::set< PONode::L > currentPolicyFringeBackup_;
  //

  // params
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
};

}

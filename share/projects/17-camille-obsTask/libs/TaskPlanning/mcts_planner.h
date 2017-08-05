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
  enum PolicySearchStateType
  {
    POLICY_INITIALIZATION = 0,
    POLICY_OPTIMIZATION,     // a sucessfull solution as been integrated ( not infinite cost )
    TERMINATED
  };

public:
  MCTSPlanner();

  // modifiers
  void setFol( const std::string & folDescription ) override;
  void solve() override;
  void integrate( const Policy::ptr & policy ) override;

  // getters
  Policy::ptr getPolicy() const override;
  MotionPlanningOrder getPlanningOrder() const override;
  bool      terminated () const override { return searchState_ == TERMINATED; }

private:
  bool solved() const { return root_->isSolved(); }
  void labelIfInfeasible( const PONode::ptr &, const PolicyNode::ptr & );

private:
  void solveFirstTime();
  void generateAlternative();

  void reinitPolicyFringe();
  void switchBackToPreviousPolicy();

  PONode::L getNodesToExpand() const;   // go along the best solution so far and accumulates the nodes that haven't been expanded, it goes up to the "deepest nodes" of the temporary path
  PONode::L getNodesToExpand( const PONode::ptr & ) const;

private:
  void checkIntegrity();

private:
  // state
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  arr bs_;

  PONode::ptr root_;

  std::list< Policy::ptr > solutions_;

  // alternative generation
  std::set< PONode::L > referencePolicyFringe_; // fringe of the last fully instanciated policy (geometric and symbolic)
  PONode::L             nextFamilyBackup_;

  PolicySearchStateType searchState_; // terminated is set to true if it is not possible to generate alternatives, fringe is empty!

  // params
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
};

}

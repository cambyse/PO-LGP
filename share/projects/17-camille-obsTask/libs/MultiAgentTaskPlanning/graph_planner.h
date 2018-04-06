#pragma once

#include <stdexcept>

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <policy.h>
#include <task_planner.h>

namespace matp
{

/*class MissingArgument: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Missing Argument";
  }
};*/

class FolFileNotFound: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Fol file not found";
  }
};

class Worlds
{
public:
  void setFol( const std::string & description );

  // getters
  bool enginesInitialized() const;
  uint agentNumber() const;
  std::vector<std::string > possibleStartStates() const { return startStates_; }
  //std::vector< double > beliefState( uint AgentId ) { return { 1.0 }; }

private:
  void parseNumberOfAgents( const std::string & description );
  void buildPossibleStartStates( const std::string & description );

private:
//  std::vector< std::shared_ptr< FOL_World > > folEngines_;
//  std::vector< double > bs_;
  uint agentNumber_ = 0;
  std::vector< std::string > startStates_;

  // constants
  const std::string agentPrefix_  = "__AGENT_";
  const std::string agentSuffix_  = "__";
  const std::string possibleFactsTag_ = "EVENTUAL_FACTS";

  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
  const mlr::String notObservableTag_ = "NOT_OBSERVABLE";
};

class GraphPlanner : public TaskPlanner
{
public:
  // modifiers
  virtual void setFol( const std::string & agentDescription ) override;
  virtual void solve() override;
  virtual void integrate( const Policy::ptr & policy ) override;

  // getters
  virtual bool terminated() const override;
  virtual Policy::ptr getPolicy() const override;
  virtual MotionPlanningOrder getPlanningOrder() const override;

  // other getters
  uint agentNumber() const { return 0;/*agents_.size();*/ }

private:
  Graph world_;
  //std::vector< Agent > agents_;
};

} // namespace matp

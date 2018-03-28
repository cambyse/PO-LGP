#pragma once

#include <stdexcept>

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <policy.h>
#include <multi_agent_task_planner.h>

namespace matp
{

class MissingArgument: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Missing Argument";
  }
};

class FolFileNotFound: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Fol file not found";
  }
};

class Agent
{
public:
  void setFols( const std::string & agentDescription );

  bool enginesInitialized() const;

  uint beliefStateSize() const { return folEngines_.size(); }

private:
  std::vector< std::shared_ptr< FOL_World > > folEngines_;
};

class GraphPlanner : public MultiAgentTaskPlanner
{
public:
  // modifiers
  virtual void setFols( const std::list< std::string > & agentDescription ) override;
  virtual void solve() override;
  virtual void integrate( const Policy::ptr & policy ) override;

  // getters
  virtual bool terminated() const override;
  virtual Policy::ptr getPolicy() const override;
  virtual MotionPlanningOrder getPlanningOrder() const override;

  // other getters
  uint agentNumber() const { return agents_.size(); }

private:
  Graph world_;
  std::vector< Agent > agents_;
};

} // namespace matp

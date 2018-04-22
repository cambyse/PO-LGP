#pragma once

#include <memory>
#include <list>

#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <constants.h>

namespace matp
{

class NotInitialized: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Object not initialized";
  }
};

class LogicEngine
{
public:
  LogicEngine() = default;
  LogicEngine( const std::string & file ) { init( file ); }
  LogicEngine( const LogicEngine & engine ); // copy ctor
  LogicEngine& operator=( const LogicEngine & engine ); // assignment operator
  ~LogicEngine() = default;

  void init( const std::string & file );
  bool initialized() const { return engine_ != nullptr; }

  uint agentNumber() const { return agentNumber_; }
  uint totalActionsNumber( uint agentId ) const;
  std::vector<FOL_World::Handle> getPossibleActions( uint agentId );
  void transition( const FOL_World::Handle & action );
  void setState( const std::string & state );
  std::string getState() const;

private:
  void parseNumberOfAgents();
  void parseActions();

private:
  std::string descriptionFile_;
  std::shared_ptr< FOL_World > engine_;
  uint agentNumber_ = 0;
  std::vector< std::vector< std::string > > actionsNames_; // per agent
};

} // namespace matp

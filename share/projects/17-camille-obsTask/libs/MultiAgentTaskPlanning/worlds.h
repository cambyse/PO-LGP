#pragma once

#include <stdexcept>

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

namespace matp
{

class IncoherentDefinition: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Incoherent definition";
  }
};

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
  std::vector< std::string > possibleStartStates() const { return startStates_; }
  std::vector< double >      egoBeliefState() { return egoBeliefState_; }
  uint actionsNumber( uint agentId ) { if( agentId >= actionsNames_.size() ) return 0; else return actionsNames_[ agentId ].size(); }

private:
  void parseNumberOfAgents( const std::string & description );
  void setUpEngine( const std::string & description );
  void retrieveAgentActions();
  void buildPossibleStartStates( const std::string & description );
  void parseBeliefState( const std::string & description );
  void checkCoherence();

  // utility
  uint getAgentId( const std::string actionName ) const;

private:
  FOL_World engine_;
  uint agentNumber_ = 0;
  std::vector< std::vector< std::string > > actionsNames_; // per agent
  std::vector< double > egoBeliefState_;
  std::vector< std::string > startStates_;

  // constants
  const std::string agentPrefix_  = "__AGENT_";
  const std::string agentSuffix_  = "__";
  const std::string possibleFactsTag_ = "EVENTUAL_FACTS";

  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
  const mlr::String notObservableTag_ = "NOT_OBSERVABLE";
};

} // namespace matp

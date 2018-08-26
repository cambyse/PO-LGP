#pragma once

#include <list>
#include <set>

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

// Utility functions
std::string actionToString( Node * action );
std::string actionToString( const FOL_World::Handle & action );
int getAgentId( const std::string actionName );
bool isOfAgent( const std::string & str, uint agentId );
bool isOfAgent( const FOL_World::Handle & action, uint agentId );
StringA nodeToStringA( Node * facts );
std::pair< std::string, std::set< std::string > > getFacts( const std::string & state );
bool isObservable( const std::string & fact );
std::string getStateStr( FOL_World & fol );
std::string concatenateFacts( const std::set< std::string > & facts );
std::set< std::string > getObservableFacts( const std::set< std::string > & facts );
std::string getObservableState( const std::string & state );
std::set< std::string > getEmergingFacts( const std::set< std::string > & intersectingFacts, const std::set< std::string > & fullFacts );

// constants
const std::string agentPrefix_     = "__AGENT_";
const std::string agentSuffix_     = "__";
const std::string possibleFactsTag_= "EVENTUAL_FACTS";
const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
const mlr::String notObservableTag_= "NOT_OBSERVABLE";
} // namespace matp

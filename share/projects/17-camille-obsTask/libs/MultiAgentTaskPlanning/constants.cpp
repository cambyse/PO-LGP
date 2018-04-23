#include <constants.h>

#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace matp
{

std::string actionToString( Node * action )
{
  std::stringstream ss;
  ss << *action;
  return ss.str();
}

std::string actionToString( const FOL_World::Handle & action )
{
  std::stringstream ss;
  ss << *action;
  return ss.str();
}

int getAgentId( const std::string actionName )
{
  //auto prefix = actionName.substr( 0, agentPrefix_.size() );
  auto prefixIndex = actionName.find( agentPrefix_ );

  if( prefixIndex == std::string::npos )
  {
    return -1;
  }

  auto prefix = actionName.substr( prefixIndex, agentPrefix_.size() );

  if( prefix != agentPrefix_ )
  {
    return -1;
  }

  auto actionNameFiltered = actionName.substr( prefixIndex + agentPrefix_.size(), actionName.size() );
  auto suffixStartIndex = actionNameFiltered.find_first_of( agentSuffix_ );
  auto idString = actionNameFiltered.substr( 0, suffixStartIndex );

  return std::stoi( idString );
}

bool isOfAgent( const std::string & str, uint agentId )
{
  auto agentPattern = agentPrefix_ + std::to_string( agentId ) + agentSuffix_;

  return str.find( agentPattern ) != -1;
}

bool isOfAgent( const FOL_World::Handle & action, uint agentId )
{
  auto actionFact = actionToString( action );

  return isOfAgent( actionFact, agentId );
}

StringA nodeToStringA( Node * facts )
{
  StringA factStringA;

  for( auto f : facts->parents )
  {
    factStringA.append( f->keys.last() );
  }

  return factStringA;
}

std::list< std::string > getFacts( const std::string & state )
{
  std::list< std::string > facts;

  using tokenizer = boost::tokenizer<boost::char_separator<char> >;

  boost::char_separator<char> sep( "," );
  tokenizer tokens( state, sep );

  for( auto fact : tokens )
  {
    boost::replace_all(fact, "{", "");
    boost::replace_all(fact, "}", "");

    facts.push_back( fact );
  }

  return facts;
}

bool isObservable( const std::string & fact )
{
  if( fact.find( notObservableTag_ ) != std::string::npos )
  {
    return false;
  }

  return true;
}

std::string getStateStr( FOL_World & fol )
{
  std::stringstream ss;
  fol.write_state( ss );

  return ss.str();
}

std::string concatenateFacts( const std::list< std::string > & facts )
{
  std::stringstream ss;
  ss << "{";
  for( auto fact : facts )
  {
    ss << fact;
  }
  ss << "}";
  return ss.str();
}

std::string getObservableState( const std::string & state )
{
  std::list< std::string > facts = getFacts( state );
  std::list< std::string > observableFacts;

  for( auto fact : facts )
  {
    if( isObservable( fact ) )
    {
      observableFacts.push_back( fact );
    }
  }

  std::string observableState = concatenateFacts( observableFacts );

  return observableState;
}

}

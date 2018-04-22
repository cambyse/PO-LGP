#include <logic_engine.h>

namespace matp
{

static std::string actionToString( Node * action )
{
  std::stringstream ss;
  ss << *action;
  return ss.str();
}

static std::string actionToString( const FOL_World::Handle & action )
{
  std::stringstream ss;
  ss << *action;
  return ss.str();
}

static int getAgentId( const std::string actionName )
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

static bool isOfAgent( const std::string & str, uint agentId )
{
  auto agentPattern = agentPrefix_ + std::to_string( agentId ) + agentSuffix_;

  return str.find( agentPattern ) != -1;
}

static bool isOfAgent( const FOL_World::Handle & action, uint agentId )
{
  auto actionFact = actionToString( action );

  return isOfAgent( actionFact, agentId );
}

// LogicEngine
LogicEngine::LogicEngine( const LogicEngine & engine ) // copy ctor
{
  if( ! engine.descriptionFile_.empty() )
  {
    init( engine.descriptionFile_ );
  }
}

LogicEngine& LogicEngine::operator=( const LogicEngine & engine ) // assignment operator
{
  if( ! engine.descriptionFile_.empty() )
  {
    init( engine.descriptionFile_ );
  }
  return *this;
}

void LogicEngine::init( const std::string & file )
{
  descriptionFile_ = file;
  engine_ = std::shared_ptr< FOL_World >( new FOL_World( FILE( descriptionFile_.c_str() ) ) );

  parseNumberOfAgents();
  parseActions();
}

uint LogicEngine::totalActionsNumber( uint agentId ) const
{
  if( agentId < actionsNames_.size() )
  {
    return actionsNames_[ 0 ].size();
  }
  else
  {
    return 0;
  }
}

std::vector<FOL_World::Handle> LogicEngine::getPossibleActions( uint agentId )
{
  if( ! initialized() ) throw NotInitialized();

  std::vector<FOL_World::Handle> filtered_actions;

  auto actions = engine_->get_actions();

  for( auto action : actions )
  {
    if( isOfAgent( action, agentId ) )
    {
      filtered_actions.push_back( action );
    }
  }

  return filtered_actions;
}

void LogicEngine::transition( const FOL_World::Handle & action )
{
  engine_->transition( action );
}

void LogicEngine::setState( const std::string & state )
{
  auto mlrState = mlr::String( state );
  engine_->set_state( mlrState );
}

std::string LogicEngine::getState() const
{
  std::stringstream ss;
  engine_->write_state( ss );
  return ss.str();
}

void LogicEngine::parseNumberOfAgents()
{
  Graph KB;
  KB.read( FILE( descriptionFile_.c_str() ) );

  uint agentIDCandidate = 1;
  std::string agentNameCandidate = agentPrefix_ + std::to_string( agentIDCandidate ) + agentSuffix_;

  while( KB[ agentNameCandidate.c_str() ] != nullptr )
  {
    agentIDCandidate++;
    agentNameCandidate = agentPrefix_ + std::to_string( agentIDCandidate ) + agentSuffix_;
  }

  agentNumber_ = agentIDCandidate;
}

void LogicEngine::parseActions()
{
  auto actions = engine_->decisionRules;

  actionsNames_ = std::vector< std::vector< std::string > > ( agentNumber_ );

  for( auto actionNode : actions )
  {
    auto actionName = actionToString( actionNode );

    auto agentId = getAgentId( actionName );

    if( agentId == -1 )
    {
      if( agentNumber_ == 1 )
      {
        agentId = 0;
      }
      else
      {
        throw IncoherentDefinition();
      }
    }

    actionsNames_[ agentId ].push_back( actionName );
  }
}

}

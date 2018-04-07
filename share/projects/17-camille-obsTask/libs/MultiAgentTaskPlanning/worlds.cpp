#include <worlds.h>

#include <boost/filesystem.hpp>

#define DEBUG(x) //x

namespace matp
{

static StringA nodeToStringA( Node * facts )
{
  StringA factStringA;

  for( auto f : facts->parents )
  {
    factStringA.append( f->keys.last() );
  }

  return factStringA;
}

static std::string getStateStr( FOL_World & fol )
{
  auto stateGraph = fol.getState();

  std::stringstream ss;
  stateGraph->write( ss );

  return ss.str();
}

static std::string actionName( Node * action )
{
  std::stringstream ss;
  ss << action->keys( 1 );
  return ss.str();
}

///////

void Worlds::setFol( const std::string & description )
{
  if( ! boost::filesystem::exists( description ) )
  {
    throw FolFileNotFound();
  }

  parseNumberOfAgents( description );

  setUpEngine( description );

  retrieveAgentActions();

  buildPossibleStartStates( description );

  parseBeliefState( description );

  checkCoherence();
}

bool Worlds::enginesInitialized() const
{
  return false;
}

uint Worlds::agentNumber() const
{
  return agentNumber_;
}

void Worlds::parseNumberOfAgents( const std::string & description )
{
  Graph KB;
  KB.read( FILE( description.c_str() ) );

  uint agentIDCandidate = 1;
  std::string agentNameCandidate = agentPrefix_ + std::to_string( agentIDCandidate ) + agentSuffix_;

  while( KB[ agentNameCandidate.c_str() ] != nullptr )
  {
    agentIDCandidate++;
    agentNameCandidate = agentPrefix_ + std::to_string( agentIDCandidate ) + agentSuffix_;
  }

  agentNumber_ = agentIDCandidate;
}

void Worlds::setUpEngine( const std::string & description )
{
  engine_.init( FILE( description.c_str() ) );
}

void Worlds::retrieveAgentActions()
{
  auto actions = engine_.decisionRules;

  actionsNames_ = std::vector< std::vector < std::string > >( agentNumber_ );

  for( auto action : actions )
  {
    auto name = actionName( action );

    if( agentNumber_ == 1 )
    {
      actionsNames_[ 0 ].push_back( name );
    }
    else
    {
      auto agentId = getAgentId( name );
      actionsNames_[ agentId ].push_back( name );
    }
  }
}

void Worlds::buildPossibleStartStates( const std::string & description )
{
  Graph KB;
  KB.read( FILE( description.c_str() ) );

  if( KB[ possibleFactsTag_.c_str() ] == nullptr )
  {
    FOL_World fol( FILE( description.c_str() ) );

    fol.reset_state();
    auto state = getStateStr( fol );
    startStates_.push_back( state );
  }
  else
  {
    // get number of possible worlds
    auto eventualFactsGraph = &KB.get<Graph>( possibleFactsTag_.c_str() );
    const uint nWorlds = eventualFactsGraph->d0;

    for( uint w = 0; w < nWorlds; w++ )
    {
      // build basic fol
      FOL_World fol( FILE( description.c_str() ) );

      // get additional facts
      auto facts = eventualFactsGraph->elem( w );

      for( auto fact : facts->graph() )
      {
        auto factStringA = nodeToStringA( fact );

        fol.addFact( factStringA );
      }

      fol.reset_state();
      auto state = getStateStr( fol );
      startStates_.push_back( state );
    }
  }
}

void Worlds::parseBeliefState( const std::string & description )
{
  Graph KB;
  KB.read( FILE( description.c_str() ) );

  if( KB[ beliefStateTag_ ] == nullptr )
  {
    egoBeliefState_.push_back( 1.0 );
  }
  else
  {
    // get number of possible worlds
    auto beliefStateGraph = &KB.get<Graph>( beliefStateTag_ );
    const uint nWorlds = beliefStateGraph->d0;

    // parse belief state
    double totalProba = 0;
    for( uint w = 0; w < nWorlds; w++ )
    {
      auto beliefNode = beliefStateGraph->elem( w );
      auto probaNode = beliefNode->graph().first();
      auto proba = probaNode->get<double>();

      egoBeliefState_.push_back( proba );

      totalProba += proba;
    }

    // check that the total sums to 1
    if( totalProba != 1.00 )
    {
      throw IncoherentDefinition();
    }
  }
}

void Worlds::checkCoherence()
{
  if( egoBeliefState_.size() != startStates_.size() )
  {
    throw IncoherentDefinition();
  }
}

// utility
uint Worlds::getAgentId( const std::string actionName ) const
{
  auto prefix = actionName.substr( 0, agentPrefix_.size() );

  if( prefix != agentPrefix_ )
  {
    throw IncoherentDefinition();
  }

  auto actionNameFiltered = actionName.substr( agentPrefix_.size(), actionName.size() );
  auto suffixStartIndex = actionNameFiltered.find_first_of( agentSuffix_ );
  auto idString = actionNameFiltered.substr( 0, suffixStartIndex );

  return std::stoi( idString );
}

} // namespace matp

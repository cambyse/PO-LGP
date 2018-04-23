#include <logic_parser.h>
#include <boost/filesystem.hpp>

#define DEBUG(x) //x

namespace matp
{

void LogicParser::parse( const std::string & description )
{
  if( ! boost::filesystem::exists( description ) )
  {
    throw FolFileNotFound();
  }

  //parseNumberOfAgents( description );

  setUpEngine( description );

  //retrieveAgentActions();

  buildPossibleStartStates( description );

  parseBeliefState( description );

  checkCoherence();

  successfullyParsed_ = true;
}

uint LogicParser::agentNumber() const
{
  return engine_.agentNumber();
}

uint LogicParser::totalActionsNumber( uint agentId ) const
{
  return engine_.totalActionsNumber( agentId );
}

void LogicParser::setUpEngine( const std::string & description )
{
  engine_.init( description );
}

void LogicParser::buildPossibleStartStates( const std::string & description )
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

void LogicParser::parseBeliefState( const std::string & description )
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

void LogicParser::checkCoherence()
{
  if( egoBeliefState_.size() != startStates_.size() )
  {
    throw IncoherentDefinition();
  }
}

} // namespace matp

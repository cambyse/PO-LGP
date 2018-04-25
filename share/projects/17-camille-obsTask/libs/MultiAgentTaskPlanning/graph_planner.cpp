#include <graph_planner.h>

#include <boost/filesystem.hpp>

namespace matp
{
// modifiers
void GraphPlanner::setFol( const std::string & descrition )
{
  if( ! boost::filesystem::exists( descrition ) ) throw FolFileNotFound();

  parser_.parse( descrition );
  graph_ = DecisionGraph( parser_.engine(), parser_.possibleStartStates(), parser_.egoBeliefState() );
}

void GraphPlanner::solve()
{

}

void GraphPlanner::integrate( const Policy::ptr & policy )
{

}

// getters
bool GraphPlanner::terminated() const
{
  return false;
}

Policy::ptr GraphPlanner::getPolicy() const
{
  return nullptr;
}

MotionPlanningOrder GraphPlanner::getPlanningOrder() const
{
  return MotionPlanningOrder( 0 );
}

void GraphPlanner::buildGraph( int maxSteps )
{
  if( ! parser_.successfullyParsed() )
  {
    return;
  }

  graph_.build( maxSteps );
}

}

#include <graph_planner.h>

#include <boost/filesystem.hpp>

#define DEBUG(x) //x

namespace matp
{

void Agent::setFols( const std::string & agentDescription )
{
  if( ! boost::filesystem::exists( agentDescription ) )
  {
    throw FolFileNotFound();
  }

  // parse agent
  //Graph Agent;
  //Agent.read( FILE( agentDescription.c_str() ) );

  // create fol
  std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
  fol->init( FILE( agentDescription.c_str() ) );

  folEngines_.push_back( fol );
}

bool Agent::enginesInitialized() const
{
  return ! folEngines_.empty();
}

// modifiers
void GraphPlanner::setFols( const std::list< std::string > & agentDescritions )
{
  if( agentDescritions.empty() ) throw MissingArgument();

  for( auto agent : agentDescritions )
  {
    Agent ag;
    ag.setFols( agent );
    agents_.push_back( ag );
  }
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

}

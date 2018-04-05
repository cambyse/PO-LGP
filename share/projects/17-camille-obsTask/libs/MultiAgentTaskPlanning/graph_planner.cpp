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

  // legacy code below
  const mlr::String notObservableTag = "NOT_OBSERVABLE";

  Graph KB;
  KB.read( FILE( agentDescription.c_str() ) );
  //KB.isDoubleLinked = false;
  // fully observable case
  if( KB[ beliefStateTag_ ] == nullptr )
  {
    // create dummy array
    folEngines_ = mlr::Array< std::shared_ptr<FOL_World> > ( 1 );
    std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
    fol->init(FILE(agentDescription.c_str()));
    folEngines_[ 0 ] = fol;
    fol->reset_state();
    //fol->KB.isDoubleLinked = false;
    // create dummy bs in observable case
    bs_ = std::vector< double >( 1 );
    bs_[ 0 ] = 1.0;
  }
  // partially observable case
  else
  {
    // get number of possible worlds
    auto bsGraph = &KB.get<Graph>( beliefStateTag_ );
    const uint nWorlds = bsGraph->d0;

    // generate all the possible fol
    folEngines_ = mlr::Array< std::shared_ptr<FOL_World> > ( nWorlds );
    bs_ = arr( nWorlds );
    for( uint w = 0; w < nWorlds; w++ )
    {
      // retrieve the facts of the belief state
      std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
      fol->init(FILE(agentDescription.c_str()));
      auto n = bsGraph->elem(w);

      std::cout << "n:" << *n << std::endl;

      // add facts
      double probability = -1;

      for( auto nn : n->graph() )
      {
        StringA fact;

        for( auto f : nn->parents )
        {
          fact.append( f->keys.last() );
        }

        if( ! fact.empty() )
        {
          // tag this fact as not observable
          StringA notObservableFact; notObservableFact.append( notObservableTag );
          for( auto s : fact ) notObservableFact.append( s );

          fol->addFact(notObservableFact);

          //std::cout << "fact:" << fact << std::endl;
          //std::cout << "notObservableFact:" << notObservableFact << std::endl;
        }
        else
        {
          probability = nn->get<double>();
          //std::cout << probability << std::endl;
        }
      }

      fol->reset_state();

      //std::cout << *fol << std::endl; // tmp
      folEngines_[ w ] = fol;
      bs_[ w ] = probability;
      folEngines_[ w ]->reset_state();
    }

    // check that the belief state sums to 1
    double total = 0;
    for( auto p : bs_ ) total += p;

    CHECK( total == 1.00, "wrong belief state definition, the total of the probabilities doesn't sum to 1" );
  }
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

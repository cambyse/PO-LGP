#include <graph_planner.h>

#include <boost/filesystem.hpp>

#define DEBUG(x) //x

namespace matp
{

void Worlds::setFol( const std::string & description )
{
  if( ! boost::filesystem::exists( description ) )
  {
    throw FolFileNotFound();
  }

  ////////////////////////////////////parse number of agents//////////////////////////////////////
  parseNumberOfAgents( description );

  //////////////////////////////////////////////////////////////////////////
  //parseBeliefStateOfAgents(  )

  /*//KB.isDoubleLinked = false;
  // fully observable case
  if( KB[ beliefStateTag_ ] == nullptr )
  {
    // create dummy array
    folEngines_ = mlr::Array< std::shared_ptr<FOL_World> > ( 1 );
    std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
    fol->init(FILE(description.c_str()));
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
      fol->init(FILE(description.c_str()));
      auto n = bsGraph->elem(w);

      //std::cout << "n:" << *n << std::endl;

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
          StringA notObservableFact; notObservableFact.append( notObservableTag_ );
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
    //////////////////////////////////////////////////////
    // check that the belief state sums to 1
    double total = 0;
    for( auto p : bs_ ) total += p;

    CHECK( total == 1.00, "wrong belief state definition, the total of the probabilities doesn't sum to 1" );
  }*/
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

// modifiers
void GraphPlanner::setFol( const std::string & agentDescrition )
{
  if( ! boost::filesystem::exists( agentDescrition ) ) throw FolFileNotFound();

//  for( auto agent : agentDescritions )
//  {
//    Agent ag;
//    ag.setFol( agent );
//    agents_.push_back( ag );
//  }
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

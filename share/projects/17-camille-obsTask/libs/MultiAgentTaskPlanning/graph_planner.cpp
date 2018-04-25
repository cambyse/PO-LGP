#include <graph_planner.h>

#include <boost/filesystem.hpp>

double m_inf() { return std::numeric_limits< double >::lowest(); }

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
  uint maxSteps = 2;

  buildGraph( 2 );

  valueIteration();
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

void GraphPlanner::valueIteration()
{
  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  double alpha = 0.5;

  values_ = std::vector< double >( graph_.size(), 0 ); // distance from root to vertex[i]


  auto comp = [ & ]( const NodeTypePtr & a, const NodeTypePtr & b ) -> bool
  {
    return values_[ a->id() ] < values_[ b->id() ];
  };

  // go from leafs to root
  auto nodes = graph_.nodes();
  auto terminals = graph_.terminalNodes();
  for( auto v : terminals )
  {
    values_[ v->id() ] = 0; // all rewards negative
  }

  // expected reward up to terminal nodes
  // add terminal nodes to Q
  uint totalUpdates = 0;
  bool stable = false;
  for( auto i = 0; ! stable && i < 1000; ++i )
  {
    for( auto u : nodes )
    {
      if( u->data().nodeType == NodeData::NodeType::ACTION )
      {
        if( u->data().agentId == 0 )
        {
          if( ! u->data().terminal )
          {
            double newValue = m_inf();

            // max operation, choose the best child
            for( auto v : u->children() )
            {
              if( values_[ v->id() ] - 1 > newValue )
              {
                newValue = values_[ v->id() ] - 1;
              }
            }

            values_[ u->id() ] = values_[ u->id() ] * ( 1 - alpha ) + alpha * newValue;
          }
        }
        else
        {
          double newValue = 0;

          uint n = u->children().size();
          for( auto v : u->children() )
          {
            newValue += 1.0 / n * values_[ v->id() ] ;
          }

          values_[ u->id() ] = values_[ u->id() ] * ( 1 - alpha ) + alpha * newValue;
        }
      }
      else if( u->data().nodeType == NodeData::NodeType::OBSERVATION )
      {
        double newValue = 0;

        for( auto v : u->children() )
        {
          newValue += v->data().p * values_[ v->id() ] ;
        }

        values_[ u->id() ] = values_[ u->id() ] * ( 1 - alpha ) + alpha * newValue;
      }
    }
  }

  std::cout << "GraphPlanner::valueIteration.. end" << std::endl;
}

}

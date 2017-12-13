/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <graph_search.h>
#include <policy_builder.h>
#include <graph_printer.h>

#include <chrono>
#include <functional>

//=====================free functions======================

namespace tp
{

void GraphSearchPlanner::setFol( const std::string & folDescription )
{
  const mlr::String notObservableTag = "NOT_OBSERVABLE";

  Graph KB;
  KB.read( FILE( folDescription.c_str() ) );
  //KB.isDoubleLinked = false;
  // fully observable case
  if( KB[ beliefStateTag_ ] == nullptr )
  {
    // create dummy array
    folEngines_ = mlr::Array< std::shared_ptr<FOL_World> > ( 1 );
    std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
    fol->init(FILE(folDescription.c_str()));
    folEngines_( 0 ) = fol;
    fol->reset_state();
    //fol->KB.isDoubleLinked = false;
    // create dummy bs in observable case
    bs_ = arr( 1 );
    bs_( 0 ) = 1.0;
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
      fol->init(FILE(folDescription.c_str()));
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
      folEngines_(w) = fol;
      bs_(w) = probability;
      folEngines_(w)->reset_state();
    }

    // check that the belief state sums to 1
    double total = 0;
    for( auto p : bs_ ) total += p;

    CHECK( total == 1.00, "wrong belief state definition, the total of the probabilities doesn't sum to 1" );
  }
}

void GraphSearchPlanner::buildGraph()
{
  buildGraphImpl();
}

void GraphSearchPlanner::solve()
{
  std::cout << "GraphSearchPlanner::solveSymbolically" << std::endl;

  if( ! graph_ )
  {
    buildGraphImpl();
  }

  /*dijkstra();
  extractSolutions();
  buildPolicy();*/

  /*Dijkstra solver( folEngines_ );
  policy_ = solver.solve( root_, terminals_ );*/

  Yens solver( folEngines_ );
  auto policies = solver.solve( graph_, 10 );

  policy_ = policies.front();
}

void GraphSearchPlanner::integrate( const Policy::ptr & policy )
{ 

}

Policy::ptr GraphSearchPlanner::getPolicy() const
{
  return policy_;
}

MotionPlanningOrder GraphSearchPlanner::getPlanningOrder() const
{
  MotionPlanningOrder po( getPolicy()->id() );

  //
  po.setParam( "type", "jointPath" );
  //

  return po;
}

void GraphSearchPlanner::saveGraphToFile( const std::string & filename )
{
  if( ! graph_ )
  {
    return;
  }

  std::ofstream file;
  file.open( filename );

  GraphPrinter printer( file );
  printer.print( graph_->root(), graph_->terminals() );

  file.close();
}

void GraphSearchPlanner::buildGraphImpl()
{
  POGraphNode::ptr root = std::make_shared< POGraphNode >( folEngines_, bs_ );

  std::queue< POGraphNode::ptr > queue;
  std::list < POGraphNode::ptr > terminals;

  queue.push( root );

  while( ! queue.empty() )
  {
    auto current = queue.front();
    queue.pop();

    if( current->isTerminal() )
    {
      std::cout << "terminal for bs:" << current->bs() << std::endl;

      terminals.push_back( current );
    }
    else
    {
      if( ! current->isExpanded() )
      {
        auto newNodes = current->expand();

        for( auto n : newNodes )
        {
          queue.push( n );
        }

        if( queue.size() % 50 == 0 )
        {
          std::cout << "queue_.size():" << queue.size() << std::endl;
        }
      }
    }
  }

  std::cout << "Graph build:" << root->shared_node_list()->size() << " number of terminal nodes:" << terminals.size() << std::endl;

  graph_ = std::make_shared< POGraph >( root, terminals );
}

void GraphSearchPlanner::yen( uint k )   // generates a set of policies
{

}

void GraphSearchPlanner::checkIntegrity()
{

}

//---------Yens--------------------//

Yens::Yens( const mlr::Array< std::shared_ptr<FOL_World> > & folEngines )
  : folEngines_( folEngines )
  , dijkstra_  ( folEngines )
{

}

static std::list< PolicyNode::ptr > serializeFrom( const PolicyNode::ptr & node )
{
  std::list< PolicyNode::ptr > nodes;

  nodes.push_back( node );

  for( auto n : node->children() )
  {
    auto newNodes = serializeFrom( n );
    nodes.insert( nodes.end(), newNodes.begin(), newNodes.end() );
  }

  return nodes;
}

static std::list< PolicyNode::ptr > serialize( const Policy::ptr & policy )
{
  return serializeFrom( policy->root() );
}

std::list< Policy::ptr > Yens::solve( const POGraph::ptr & graph, const uint k )
{
  graph_ = graph;

  std::list< Policy::ptr > policies;  // A
  std::list< Policy::ptr > altPolicies; // B

  auto policy_0 = dijkstra_.solve( graph, graph->root() );
  policies.push_back( policy_0 );

  // create the mask of edges to remove
  auto mask = std::make_shared< GraphEdgeRewards >( graph );

  auto lastPolicy = policy_0;
  for( auto l = 1; l < k; ++l )
  {
    // serialize the solution
    auto s_lastPolicy = serialize( lastPolicy );

    for( auto i = 3; i < s_lastPolicy.size(); ++i ) ///*auto sit = std::begin( s_lastPolicy ); sit != std::end( s_lastPolicy ); ++sit*/ ) // s is the spur node
    {
      auto spurNodeIt = s_lastPolicy.begin();
      std::advance( spurNodeIt, i );
      auto spurNode   = *spurNodeIt;
      auto rootPath = std::list< PolicyNode::ptr >( std::begin( s_lastPolicy ), spurNodeIt );

      for( auto previousPolicy : policies )
      {
        auto s_previousPolicy = serialize( previousPolicy );
        auto ithNodeIt = s_previousPolicy.begin();
        std::advance( ithNodeIt, i );
        auto previousRootPath = std::list< PolicyNode::ptr >( std::begin( s_previousPolicy ), ithNodeIt );

        if( rootPath == previousRootPath )
        {
          // Remove the links that are part of the previous shortest paths which share the same root path.
          auto from = (*ithNodeIt)->id();
          auto to   = (*(++ithNodeIt))->id();

          mask->removeEdge( from, to );
        }

        for( auto n : rootPath )
        {
          // Remove n
          if( n->id() != spurNode->id() )
          {
            mask->removeNode( n->id() );
          }
        }

        auto spurPolicy = dijkstra_.solve( graph, graph->getNode( spurNode->id() ), mask );

        if( spurPolicy )
        {
          auto altPolicy = fuse( lastPolicy, spurPolicy );

          altPolicies.push_back( altPolicy );
        }
      }

      // reset
      mask->reset();
    }
  }

  return policies;
}

}

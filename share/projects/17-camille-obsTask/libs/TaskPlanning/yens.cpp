#include <yens.h>

namespace tp {

//---------Yens--------------------//

Yens::Yens( const mlr::Array< std::shared_ptr<FOL_World> > & folEngines )
  : folEngines_( folEngines )
  , dijkstra_  ( folEngines )
{

}

/*
std::list< Policy::ptr > Yens::solve( const POGraph::ptr & graph, const uint K )
{
  graph_ = graph;

  std::list< Policy::ptr > policies;    // A
  std::list< Policy::ptr > altPolicies; // B
  std::list< Policy::ptr > lastPoliciesFIFO;

  auto policy_0 = dijkstra_.solve( graph, graph->root() );
  policies.push_back( policy_0 );

  // create the mask of edges to remove
  auto mask = std::make_shared< GraphEdgeRewards >( graph );

  lastPoliciesFIFO.push_back( policy_0 );

  for( auto k = 1; k <= K; ++k )
  {
    std::cout << "k=" << k << std::endl;

    auto fifo = lastPoliciesFIFO;
    lastPoliciesFIFO.clear();

    while( ! fifo.empty() )
    {
      auto lastPolicy = fifo.back();
      fifo.pop_back();

      // serialize the solution
      auto lastPolicySegmented = segment( lastPolicy );

      for( auto segId = 0; segId < lastPolicySegmented.size(); ++segId )
      {
        std::cout << "segId=" << segId << std::endl;

        auto seg = lastPolicySegmented[ segId ];

        for( auto i = 0; i < seg.size(); ++i )
        {
          std::cout << "i=" << i << std::endl;

          auto spurNodeIt = seg.begin();
          std::advance( spurNodeIt, i );
          auto spurNode   = *spurNodeIt;
          auto rootPath = std::list< PolicyNode::ptr >( std::begin( seg ), spurNodeIt );

          std::cout << "spurnode=" << spurNode->id() << std::endl;

          if( ! spurNode->children().empty() )
          {
            for( auto previousPolicy : policies )
            {
              auto previousPolicySegmented = segment( previousPolicy );
              auto previousPolicySeg = previousPolicySegmented[ segId ];
              auto ithNodeIt = previousPolicySeg.begin();
              std::advance( ithNodeIt, i );
              auto previousRootPath = std::list< PolicyNode::ptr >( std::begin( previousPolicySeg ), ithNodeIt );

              if( equivalent( rootPath, previousRootPath ) )
              {
                // Remove the choice that has already been done by the previous policies at the spur node
                // Remove the links that are part of the previous shortest paths which share the same root path.
                auto fromNode = (*ithNodeIt);
                //auto to   = (*(++ithNodeIt))->id();
                for( auto c : fromNode->children() )
                {
                  auto from = fromNode->id();
                  auto to   = c->id();

                  std::cout << "delete edge:" << from << "-" << to << std::endl;

                  mask->removeEdge( from, to );
                }
              }

              // Remove the the root path
              for( auto n : rootPath )
              {
                // Remove n
                if( n->id() != spurNode->id() )
                {
                  mask->removeNode( n->id() );
                }
              }
            }

            auto spurPolicy = dijkstra_.solve( graph, graph->getNode( spurNode->id() ), mask );

            if( spurPolicy )
            {
              std::cout << "alternative policy found!" << std::endl;

              auto altPolicy = fuse( lastPolicy, spurPolicy );

              altPolicies.push_back( altPolicy );
            }
          }

          // reset
          mask->reset();
        }

        // add the best alternative policy
        std::cout << altPolicies.size() << " alternative policies for k=" << k << " segId=" << segId << std::endl;
        auto policyComp = []( const Policy::ptr & a, const Policy::ptr & b )
        {
          return a->cost() < b->cost();
        };

        altPolicies.sort( policyComp );

        if( ! altPolicies.empty() )
        {
          auto newPolicy = altPolicies.front();
          altPolicies.pop_front();

          policies.push_back( newPolicy );
          lastPoliciesFIFO.push_back( newPolicy );
        }
      }
    }
  }

  return policies;
}*/

std::list< Policy::ptr > Yens::solve( const POGraph::ptr & graph, const uint K )
{
  graph_ = graph;

  // sort policies
  const auto policyComp = []( const Policy::ptr & a, const Policy::ptr & b )
  {
    return a->expectedSymReward() > b->expectedSymReward();
  };
  //

  std::list< Policy::ptr > policies;    // A
  std::list< Policy::ptr > altPolicies; // B

  auto policy_0 = dijkstra_.solve( graph, graph->root() );
  policies.push_back( policy_0 );

  // create the mask of edges to remove
  auto mask = std::make_shared< GraphEdgeRewards >( graph );

  for( auto k = 1; k <= K; ++k )
  {
    //std::cout << "k=" << k << std::endl;

    auto lastPolicy = policies.back();

    // serialize the solution
    auto s_lastPolicy = serialize( lastPolicy );

    for( auto i = 0; i < s_lastPolicy.size(); ++i )
    {
      std::cout << "i=" << i << std::endl;

      auto spurNodeIt = s_lastPolicy.begin();
      std::advance( spurNodeIt, i );
      auto spurNode   = *spurNodeIt;
      auto rootPath = std::list< PolicyNode::ptr >( std::begin( s_lastPolicy ), spurNodeIt );

      //std::cout << "spurnode=" << spurNode->id() << std::endl;

      if( ! spurNode->children().empty() )
      {
        for( auto previousPolicy : policies )
        {
          auto s_previousPolicy = serialize( previousPolicy );
          auto ithNodeIt = s_previousPolicy.begin();
          std::advance( ithNodeIt, i );
          auto previousRootPath = std::list< PolicyNode::ptr >( std::begin( s_previousPolicy ), ithNodeIt );

          if( equivalent( rootPath, previousRootPath ) )
          {
            // Remove the choice that has already been done by the previous policies at the spur node
            // Remove the links that are part of the previous shortest paths which share the same root path.
            auto fromNode = (*ithNodeIt);
            //auto to   = (*(++ithNodeIt))->id();
            for( auto c : fromNode->children() )
            {
              auto from = fromNode->id();
              auto to   = c->id();

              std::cout << "delete edge:" << from << "-" << to << std::endl;

              mask->removeEdge( from, to );
            }
          }

          // Remove the the root path
          for( auto n : rootPath )
          {
            // Remove n
            if( n->id() != spurNode->id() )
            {
              // remove node and sibling
              auto parent = n->parent();
              if( ! parent )
              {
                mask->removeNode( n->id() );
              }
              else
              {
                for( auto n : parent->children() )
                {
                  mask->removeNode( n->id() );
                }
              }
            }
          }
        }

        auto spurPolicy = dijkstra_.solve( graph, graph->getNode( spurNode->id() ), mask );

        if( spurPolicy )
        {
          //std::cout << "alternative policy found!" << std::endl;

          auto altPolicy = fuse( lastPolicy, spurPolicy );

          altPolicies.push_back( altPolicy );
        }
      }

      // reset
      mask->reset();
    }

    // add the best alternative policy
    //std::cout << altPolicies.size() << " alternative policies for k=" << k << std::endl;
    altPolicies.sort( policyComp );

    if( ! altPolicies.empty() )
    {
      policies.push_back( altPolicies.front() );
      altPolicies.pop_front();
    }
  }

  return policies;
}

}
